"""
Ras_output.py
==============
Drive/Steer/steer_control.py 와 Drive/Throttle/throttle_control.py가 각자 계산해서
기록해둔 최종 조향각(steer_output.json)과 최종 속도(throttle_output.json)를 읽어서,
STM32가 이해하는 CAN 프로토콜로 변환한 뒤 전송만 하는 출력 전용 계층입니다.

이 파일은 "무엇을 할지"는 전혀 판단하지 않습니다 (그건 Steer/Throttle의 몫). 오직:
  1) steer_output.json / throttle_output.json 을 읽고
  2) 단위를 변환하고 (조향각: -MAX_STEERING_DEG~+MAX_STEERING_DEG도 -> STM32 30~150)
  3) CAN(0x100)으로 STM32에 실어 보내는 일만 합니다.

프로토콜은 rc car.py / navis_ros2_ws의 can_bridge.py와 완전히 동일하게 맞춘다
(0x100 DRIVE_CMD: speed int16 LE + angle uint16 LE, 500kbps, 0x200 STATUS로 STM32가 응답).

[입력 파일이 오래됐거나 없을 때]
Steer/Throttle 프로세스가 죽었거나 응답이 없으면(OUTPUT_STALE_TIMEOUT_S 초과) 각각
조향은 중립(0도=정면), 속도는 0(정지)으로 대체합니다. Steer/Throttle 자체의 안전 로직
(카메라/라이다 신선도 체크, 장애물 안전정지)과는 별개로, "그 프로세스 자체가 죽었을 때"를
대비한 이 계층만의 마지막 방어선입니다.

[STM32 명령-실측 워치독]
STM32가 0x200으로 보내는 실제 speed/angle(STM32가 실제로 적용한 값)을 백그라운드 스레드로
계속 수신해서, 방금 보낸 명령값과 비교합니다. 차이가 STATUS_MISMATCH_PERSIST_S 이상 계속
벌어져 있으면(가감속/조향 램프 중의 일시적인 지연은 정상이라 무시) 기계적 걸림/모터 고장/배선
문제로 보고 강제로 정지 명령을 보냅니다. STM32 응답 자체가 STATUS_RECV_TIMEOUT_S 동안 아예
없어도 통신 두절로 보고 같은 방식으로 정지합니다. 이건 "어디로 갈지"를 다시 계산하는 게
아니라(그건 Path_Planning/Explore_Map 몫), 하드웨어가 명령대로 실제 움직이고 있는지만
감시하는 최후 안전망입니다.

[수동 비상정지(킬스위치) - 가장 높은 우선순위]
Drive/OutInterface/kill_switch.py 가 기록하는 state/estop.json 을 매 사이클 확인합니다.
active=true 이면 Steer/Throttle 계산 결과나 STM32 워치독 판정과 무관하게 무조건 속도를
0으로 강제합니다. 소프트웨어 판단 전체를 우회하는, 사람이 직접 개입하는 최종 안전장치입니다.
"""

import json
import os
import struct
import threading
import time

import can

# ===== 경로 (Testing/ 폴더를 통째로 옮겨도 항상 Testing/state 를 가리키도록 상대경로 계산) =====
# 이 파일 위치: Testing/Drive/OutInterface/Ras_output.py -> 세 단계 위가 Testing/
TESTING_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STATE_DIR = os.path.join(TESTING_DIR, "state")
STEER_OUTPUT_FILE = os.path.join(STATE_DIR, "steer_output.json")     # Drive/Steer/steer_control.py가 씀
THROTTLE_OUTPUT_FILE = os.path.join(STATE_DIR, "throttle_output.json")  # Drive/Throttle/throttle_control.py가 씀
ESTOP_FILE = os.path.join(STATE_DIR, "estop.json")                   # Drive/OutInterface/kill_switch.py가 씀

# ===== CAN 프로토콜 상수 (rc car.py / can_bridge.py 와 반드시 동일하게 유지) =====
CAN_ID_DRIVE_CMD = 0x100   # 라파 -> STM32 : 주행 명령
CAN_ID_STATUS = 0x200      # STM32 -> 라파 : 상태 응답 (이 스크립트에서는 사용하지 않음)
SPEED_MIN, SPEED_MAX = -100, 100
ANGLE_MIN, ANGLE_MAX = 30, 150
ANGLE_CENTER = 90

MAX_STEERING_DEG = 30.0    # Steer 쪽과 동일하게 맞춰야 함 (도 단위 -> CAN angle 변환 기준)

# steer_output.json/throttle_output.json은 Steer/Throttle이 50Hz(0.02s)로 갱신한다.
# 그보다 훨씬 여유 있게 잡아서(잠깐의 스케줄링 지연은 넘기되) 프로세스가 실제로 죽은
# 경우는 확실히 걸러내는 값으로 설정.
OUTPUT_STALE_TIMEOUT_S = 0.3

CONTROL_PERIOD_S = 0.02    # 약 50Hz

# ===== STM32 명령-실측 워치독 설정 =====
# STM32도 100ms마다 자동으로 0x200을 보내므로(rc car.py 참고), 이 시간 동안 응답이
# 아예 없으면 통신 두절로 간주.
STATUS_RECV_TIMEOUT_S = 3.0
# speed(-100~100)/angle(30~150) 단위 명령-실측 허용 오차. 하드웨어 특성(모터/서보 반응속도)에
# 맞춰 실기 테스트하면서 다시 튜닝이 필요한 잠정값.
STATUS_MISMATCH_SPEED_TOLERANCE = 15
STATUS_MISMATCH_ANGLE_TOLERANCE = 15
# 이 시간 이상 계속 어긋나야 고장으로 판단 (가감속/조향 램프 중의 일시적 차이는 정상이므로 무시)
STATUS_MISMATCH_PERSIST_S = 1.0


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _read_output_file(path, key):
    """{key:..., "timestamp":...} 파일을 읽음. 없거나 깨졌거나 오래되면 None."""
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        value, timestamp = data[key], data["timestamp"]
    except (json.JSONDecodeError, KeyError, OSError):
        return None
    if (time.time() - timestamp) > OUTPUT_STALE_TIMEOUT_S:
        return None
    return value


def read_steer_output():
    """steer_control.py가 계산한 최종 조향각(도). 없거나 오래되면 0.0(중립)."""
    angle_deg = _read_output_file(STEER_OUTPUT_FILE, "angle_deg")
    return 0.0 if angle_deg is None else angle_deg


def read_throttle_output():
    """throttle_control.py가 계산한 최종 speed_percent. 없거나 오래되면 0.0(정지)."""
    speed_percent = _read_output_file(THROTTLE_OUTPUT_FILE, "speed_percent")
    return 0.0 if speed_percent is None else speed_percent


def read_estop_active():
    """kill_switch.py가 기록하는 수동 비상정지 상태. 파일이 없으면(킬스위치를 안 띄웠으면)
    기본값 False - 신선도(timestamp) 체크는 하지 않는다: 킬스위치 프로세스가 죽었을 때
    active=True로 멈춰있는 상태로 굳는 게(안전 쪽으로 fail) active=False로 풀려버리는
    것보다 낫기 때문."""
    if not os.path.exists(ESTOP_FILE):
        return False
    try:
        with open(ESTOP_FILE) as f:
            return bool(json.load(f).get("active", False))
    except (json.JSONDecodeError, OSError):
        return False


def deg_to_can_angle(angle_deg):
    """조향각(-MAX_STEERING_DEG~+MAX_STEERING_DEG, 0=정면) -> STM32 CAN angle(30~150, 90=정면)."""
    angle_deg = _clamp(angle_deg, -MAX_STEERING_DEG, MAX_STEERING_DEG)
    scale = (ANGLE_MAX - ANGLE_CENTER) / MAX_STEERING_DEG   # 60 / 30 = 2.0
    return ANGLE_CENTER + angle_deg * scale


class Stm32Link:
    """STM32와 CAN(0x100)으로 통신하는 래퍼. rc car.py의 NavisCar와 동일한 프로토콜.
    백그라운드 스레드로 0x200 상태 응답을 계속 수신해서 명령-실측 워치독도 담당한다."""

    def __init__(self, channel="can0"):
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        self._last_speed = 0
        self._last_angle = ANGLE_CENTER

        self._status_lock = threading.Lock()
        self._last_status = None       # (speed, angle) - STM32가 실제로 보고한 값
        self._last_status_time = None
        self._mismatch_since = None    # 명령-실측 불일치가 시작된 시각 (없으면 None)

        self._stop_recv = threading.Event()
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

    def _receive_loop(self):
        """0x200(STATUS)을 계속 수신해서 최신 실측값을 저장 - send_drive()와 별도 스레드라
        50Hz 송신 루프를 블로킹하지 않음."""
        while not self._stop_recv.is_set():
            try:
                msg = self.bus.recv(timeout=0.5)
            except Exception:
                continue
            if msg is None or msg.arbitration_id != CAN_ID_STATUS or len(msg.data) < 4:
                continue
            speed, angle = struct.unpack("<hH", bytes(msg.data[:4]))
            with self._status_lock:
                self._last_status = (speed, angle)
                self._last_status_time = time.time()

    def send_drive(self, speed_percent, can_angle):
        speed = int(round(_clamp(speed_percent, SPEED_MIN, SPEED_MAX)))
        angle = int(round(_clamp(can_angle, ANGLE_MIN, ANGLE_MAX)))
        data = struct.pack("<hH", speed, angle)
        msg = can.Message(arbitration_id=CAN_ID_DRIVE_CMD, data=data, is_extended_id=False)
        self.bus.send(msg)
        self._last_speed, self._last_angle = speed, angle

    def check_watchdog(self):
        """
        직전에 보낸 명령값과 STM32가 보고한 실측값을 비교.
        반환: (ok: bool, reason: str). ok=False면 호출부가 강제로 정지시켜야 함.
        """
        now = time.time()
        with self._status_lock:
            status, status_time = self._last_status, self._last_status_time

        if status_time is None or (now - status_time) > STATUS_RECV_TIMEOUT_S:
            self._mismatch_since = None
            return False, "STM32 응답(0x200) 없음 - 통신 두절 의심"

        actual_speed, actual_angle = status
        mismatched = (abs(actual_speed - self._last_speed) > STATUS_MISMATCH_SPEED_TOLERANCE or
                      abs(actual_angle - self._last_angle) > STATUS_MISMATCH_ANGLE_TOLERANCE)

        if not mismatched:
            self._mismatch_since = None
            return True, "ok"

        if self._mismatch_since is None:
            self._mismatch_since = now
            return True, "ok"  # 방금 시작된 불일치는 가감속/조향 램프 중일 수 있어 아직 고장으로 안 봄

        if (now - self._mismatch_since) >= STATUS_MISMATCH_PERSIST_S:
            return False, (f"명령값(speed={self._last_speed}, angle={self._last_angle})과 "
                            f"실측값(speed={actual_speed}, angle={actual_angle}) 불일치가 "
                            f"{STATUS_MISMATCH_PERSIST_S:.1f}s 이상 지속 - 기계적 걸림/모터 고장 의심")
        return True, "ok"

    def close(self):
        self._stop_recv.set()
        try:
            self.send_drive(0, self._last_angle)   # 종료 전 정지 명령 먼저 (차 폭주 방지)
        except Exception:
            pass
        self.bus.shutdown()


def main():
    print("STM32 출력(Ras_output) 시작 - CAN 채널 can0 (Ctrl+C 종료)")
    link = Stm32Link(channel="can0")
    was_ok = True   # reason 문자열엔 실측값이 섞여 매 사이클 바뀔 수 있어, ok/not ok "전환" 기준으로만 출력
    was_estopped = False
    try:
        while True:
            angle_deg = read_steer_output()
            speed_percent = read_throttle_output()
            can_angle = deg_to_can_angle(angle_deg)

            ok, reason = link.check_watchdog()
            if not ok:
                speed_percent = 0.0   # 워치독 발동 - 조향은 유지하고 속도만 강제 정지 (급조향 방지)
            if ok != was_ok:
                print(f"[워치독] {reason}" if not ok else "[워치독] 정상 복귀")
                was_ok = ok

            estopped = read_estop_active()
            if estopped:
                speed_percent = 0.0   # 수동 비상정지 - Steer/Throttle/워치독 판정과 무관하게 최우선 강제 정지
            if estopped != was_estopped:
                print("[킬스위치] *** 수동 비상정지 발동 ***" if estopped else "[킬스위치] 수동 비상정지 해제")
                was_estopped = estopped

            link.send_drive(speed_percent, can_angle)

            time.sleep(CONTROL_PERIOD_S)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        link.close()


if __name__ == "__main__":
    main()
