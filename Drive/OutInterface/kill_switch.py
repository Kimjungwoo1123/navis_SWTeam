"""
kill_switch.py
================
사람이 소프트웨어 판단을 전부 무시하고 즉시 차를 멈출 수 있는 수동 비상정지 스위치.

이 스크립트를 터미널에서 실행해두면, 아무 키나 누를 때마다 state/estop.json 의
active 값이 켜짐/꺼짐으로 토글됩니다. Ras_output.py는 매 사이클(50Hz) 이 파일을
확인해서 active=true 이면 Steer/Throttle/Path_Planning이 뭐라고 계산했든, 심지어
STM32 워치독 판정과도 무관하게 무조건 속도 0을 STM32에 보냅니다 - 이 레포에서 가장
우선순위가 높은 정지 수단입니다.

[사용법]
    python3 kill_switch.py
    (run_all.py/run_explore.py가 다른 프로세스들과 함께 자동으로 띄웁니다.)

아무 키나 누르면 즉시정지 발동, 다시 누르면 해제. Ctrl+C로 이 스위치 자체를 종료하면
(터미널을 닫는 것도 포함) 안전하게 estop을 해제한 뒤 끝나는데, run_all.py 입장에서는
"킬스위치 프로세스가 예기치 않게 죽었다"로 감지되어 나머지 프로세스도 전부 안전 종료됩니다
- 킬스위치 담당자가 자리를 비우면 시스템 전체가 멈추는 것도 의도된 안전장치입니다.
"""

import json
import os
import select
import sys
import termios
import time
import tty

# Testing/ 폴더를 통째로 옮겨도 항상 Testing/state 를 가리키도록 스크립트 위치 기준 상대경로로 계산
# (이 파일 위치: Testing/Drive/OutInterface/kill_switch.py -> 세 단계 위가 Testing/)
TESTING_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STATE_DIR = os.path.join(TESTING_DIR, "state")
ESTOP_FILE = os.path.join(STATE_DIR, "estop.json")


def publish_estop(active):
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp_path = ESTOP_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump({"active": active, "timestamp": time.time()}, f)
    os.replace(tmp_path, ESTOP_FILE)


def main():
    active = False
    publish_estop(False)  # 시작할 땐 항상 해제 상태로 초기화

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    print("=== 수동 비상정지 스위치 ===")
    print("아무 키나 누르면 즉시 정지 / 다시 누르면 해제 (Ctrl+C: 이 스위치 자체 종료)")
    try:
        while True:
            dr, _, _ = select.select([sys.stdin], [], [], 0.2)
            if dr:
                sys.stdin.read(1)
                active = not active
                publish_estop(active)
                print("*** 비상정지 발동 - STM32에 정지 명령 강제 전송 중 ***" if active
                      else "--- 비상정지 해제 ---")
    except KeyboardInterrupt:
        pass
    finally:
        publish_estop(False)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\n킬스위치 종료 (비상정지 해제됨)")


if __name__ == "__main__":
    main()
