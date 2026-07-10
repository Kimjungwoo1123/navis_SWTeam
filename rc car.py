"""
NAVIS RC카 - 라즈베리파이 CAN 제어 파이썬 래퍼
STM32(can_comm.c) 프로토콜:
  0x100 (DRIVE_CMD) Pi->STM32 : speed(int16 LE) + angle(uint16 LE)
  0x200 (STATUS)    STM32->Pi : 동일 포맷, 500kbps
"""
# ↑ 이 파일이 뭐 하는지 설명하는 docstring.
#   STM32랑 주고받는 규칙: 0x100 ID로 명령 보내고, 0x200 ID로 상태 받음.
#   speed는 int16(부호O), angle은 uint16(부호X), 둘 다 리틀엔디안(LE), 통신속도 500kbps.

import struct   # 숫자 <-> 바이트 변환용 (파이썬 int를 CAN이 이해하는 바이트로 포장)
import time     # 시간 측정/대기용 (read_status의 타임아웃에 사용)
import can      # python-can 라이브러리 (실제 CAN 통신 담당). sudo apt install python3-can

# ===== 통신 규약 상수 (STM32 코드와 반드시 똑같이 맞춰야 함) =====
CAN_ID_DRIVE_CMD = 0x100   # 라파 -> STM32 : 주행 명령을 보낼 때 쓰는 CAN ID
CAN_ID_STATUS    = 0x200   # STM32 -> 라파 : STM32가 현재 상태를 알려줄 때 쓰는 ID

# ===== 값 범위 (STM32 actuator.c 기준. 이 밖의 값은 잘라냄) =====
SPEED_MIN, SPEED_MAX = -100, 100   # 속도: -100(최대후진) ~ 100(최대전진)
ANGLE_MIN, ANGLE_MAX = 30, 150     # 조향: 30(좌끝) ~ 150(우끝)
ANGLE_CENTER = 90                  # 정면(핸들 중앙)


def _clamp(v, lo, hi):
    """값 v를 [lo, hi] 범위 안으로 강제로 집어넣는다.
       예: _clamp(200, 30, 150) -> 150 / _clamp(-5, 0, 10) -> 0
       (앞에 _ 붙은 건 '내부용 함수'라는 파이썬 관례. 밖에서 직접 안 씀)"""
    return max(lo, min(hi, v))
    # min(hi, v): v가 hi보다 크면 hi로 / max(lo, ...): 그 결과가 lo보다 작으면 lo로


class NavisCar:
    """RC카를 CAN으로 제어하는 리모컨 클래스.
       소프트웨어팀은 이 클래스의 함수(drive, stop 등)만 쓰면 됨."""

    def __init__(self, channel="can0"):
        """객체를 만들 때 자동 실행됨 (car = NavisCar() 하는 순간).
           channel : CAN 인터페이스 이름 (기본 can0)"""
        # SocketCAN 버스에 연결. can0가 미리 켜져(up) 있어야 함.
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        # 마지막으로 보낸 값을 기억해둠 (steer/set_speed에서 한쪽만 바꿀 때 씀)
        self._last_speed = 0            # 마지막 속도 (처음엔 정지)
        self._last_angle = ANGLE_CENTER # 마지막 각도 (처음엔 정면 90)

    def drive(self, speed, angle):
        """속도와 각도를 '동시에' 지정해서 STM32로 보낸다. (가장 핵심 함수)"""
        # 1) 입력값을 정수로 바꾸고 허용 범위로 자름 (이상한 값 방지)
        speed = _clamp(int(speed), SPEED_MIN, SPEED_MAX)
        angle = _clamp(int(angle), ANGLE_MIN, ANGLE_MAX)

        # 2) 숫자를 CAN이 이해하는 4바이트로 포장
        #    "<hH" 의미: < = 리틀엔디안, h = int16(speed), H = uint16(angle)
        #    예: speed=50, angle=120 -> b'\x32\x00\x78\x00'
        data = struct.pack("<hH", speed, angle)

        # 3) CAN 메시지 객체 생성
        msg = can.Message(arbitration_id=CAN_ID_DRIVE_CMD,  # 0x100으로 보냄
                          data=data,
                          is_extended_id=False)  # 표준 11비트 ID 사용 (확장 ID 아님)

        # 4) 실제 전송
        self.bus.send(msg)

        # 5) 방금 보낸 값을 기억 (다음에 steer/set_speed 쓸 때 참조)
        self._last_speed, self._last_angle = speed, angle
        return speed, angle   # 실제로 보낸 값 반환 (범위 잘렸는지 확인용)

    def set_speed(self, speed):
        """속도만 바꾸고 조향 각도는 마지막 값 그대로 유지."""
        return self.drive(speed, self._last_angle)  # 각도 자리에 마지막 각도 넣음

    def steer(self, angle):
        """조향만 바꾸고 속도는 마지막 값 그대로 유지."""
        return self.drive(self._last_speed, angle)  # 속도 자리에 마지막 속도 넣음

    def stop(self):
        """정지. 속도만 0으로, 조향은 유지."""
        return self.drive(0, self._last_angle)

    def read_status(self, timeout=0.5):
        """STM32가 보내는 최신 상태(0x200)를 하나 읽어서 반환.
           반환: {'speed': .., 'angle': ..}  또는  못 받으면 None
           STM32는 100ms마다 자동 송신하므로 0.5초면 충분히 받음."""
        deadline = time.time() + timeout   # 언제까지 기다릴지 마감시각 계산
        while time.time() < deadline:      # 마감 전까지 반복
            # 버스에서 메시지 하나 받기 (남은 시간만큼만 대기)
            msg = self.bus.recv(timeout=deadline - time.time())
            if msg is None:                # 시간 안에 아무것도 안 오면
                break                      # 반복 종료
            # 받은 게 STM32 상태(0x200)이고 4바이트 이상이면 파싱
            if msg.arbitration_id == CAN_ID_STATUS and len(msg.data) >= 4:
                # 바이트를 다시 숫자로 풀기 (drive의 pack과 반대 작업)
                speed, angle = struct.unpack("<hH", bytes(msg.data[:4]))
                return {"speed": speed, "angle": angle}  # dict로 반환
            # 0x200이 아닌 다른 메시지면 무시하고 계속 반복
        return None   # 끝까지 0x200 못 받으면 None

    def close(self):
        """연결 종료. 안전을 위해 정지 명령을 먼저 보낸 뒤 버스를 닫음."""
        try:
            self.drive(0, self._last_angle)  # 먼저 멈춤 (차 폭주 방지)
        except Exception:
            pass                             # 정지 실패해도 무시하고 닫기 진행
        self.bus.shutdown()                  # CAN 버스 연결 해제

    # ----- with 문 지원용 (아래 두 함수 덕분에 with NavisCar() as car: 가능) -----
    def __enter__(self):
        """with 블록 시작할 때 자동 호출. car 객체를 돌려줌."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """with 블록 끝날 때 자동 호출 (에러로 끝나도 호출됨).
           -> 여기서 close()를 불러서 자동 정지 + 종료. 그래서 with가 안전함."""
        self.close()


# ===== 이 파일을 '직접 실행'했을 때만 도는 데모 =====
# (python3 navis_car.py 로 실행하면 O / 다른 파일이 import 하면 X)
if __name__ == "__main__":
    print("=== NAVIS Car CAN 래퍼 데모 ===")
    with NavisCar() as car:            # with 사용 -> 끝나면 자동 정지/종료
        # 1) 먼저 STM32 상태 읽어서 통신 되는지 확인
        st = car.read_status()
        if st is None:
            print("[경고] STM32 상태(0x200) 수신 안 됨. 전원/배선 확인.")
        else:
            print(f"[수신] 현재: speed={st['speed']}, angle={st['angle']}")

        # 2) 조향만 좌 -> 우 -> 정면으로 흔들기 (모터는 정지 상태라 안전)
        print("[송신] 조향: 좌(38) -> 우(146) -> 정면(90)")
        car.steer(38);  time.sleep(1)  # 좌로 꺾고 1초 대기
        car.steer(146); time.sleep(1)  # 우로 꺾고 1초 대기
        car.steer(90);  time.sleep(1)  # 정면으로 돌리고 1초 대기

        # 3) 값이 실제로 바뀌었는지 다시 상태 읽어 확인
        st = car.read_status()
        if st:
            print(f"[수신] 갱신: speed={st['speed']}, angle={st['angle']}")

        print("[송신] 정지")
        car.stop()
    # with 블록을 벗어나면 __exit__ -> close()가 자동 실행됨
    print("종료.")