"""
throttle_control.py
=====================
자율주행 차량의 목표 주행 속도를 결정합니다.
Path_Planning 쪽에서 계산한 목표 속도(SPEED_STATE_FILE)를 읽어서 최종 속도를 정하는
역할만 담당하고, "얼마나 빠르게 갈지/멈출지"는 상위(Path_Planning) 판단을 그대로 따릅니다.

[안전 반사 (Path_Planning과는 독립적인 최후 방어선)]
장애물을 피해서 "어디로 갈지"를 다시 계산하는 판단은 전적으로 Path_Planning의 몫이고,
여기서는 그 결과(speed_percent)를 그대로 따른다. 다만 Path_Planning이 같이 실어 보내는
min_obstacle_dist_mm 값이 SAFETY_STOP_DIST_MM보다 가까우면, speed_percent가 뭐라고 되어
있든 상관없이 이 모듈이 자체적으로 강제 정지한다. Path_Planning 쪽 판단 로직에 버그가 있거나
응답이 늦어지는 경우를 대비한 최후 안전망이다.

[출력]
실제 모터 구동은 이 스크립트가 아니라 STM32가 담당하고, 라즈베리파이는 CAN으로 명령만
넘깁니다(Drive/OutInterface/Ras_output.py). 그래서 이 스크립트는 GPIO를 전혀 건드리지 않고,
급가속/급제동 방지용 가감속 램프(ACCEL_STEP_PERCENT)만 적용한 뒤 매 주기 최종 speed_percent를
THROTTLE_OUTPUT_FILE(throttle_output.json)에 기록만 합니다. Ras_output.py가 이 값을 읽어서
STM32 CAN 프로토콜(speed -100~100)로 그대로 실어 전송합니다.
"""

import json
import os
import time

# Testing/ 폴더를 통째로 옮겨도 항상 Testing/state 를 가리키도록 스크립트 위치 기준 상대경로로 계산
# (이 파일 위치: Testing/Drive/Throttle/throttle_control.py -> 두 단계 위가 Testing/)
TESTING_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STATE_DIR = os.path.join(TESTING_DIR, "state")
SPEED_STATE_FILE = os.path.join(STATE_DIR, "speed_command.json")
THROTTLE_OUTPUT_FILE = os.path.join(STATE_DIR, "throttle_output.json")

ACCEL_STEP_PERCENT = 2.0       # 제어 주기당 speed 변화 제한폭 (급가속/급제동 방지)
CONTROL_PERIOD_S = 0.02        # 약 50Hz

# Path_Planning은 매 프레임 도는 Camera와 달리 사이클 자체가 느림(위치추정 포함).
# 실측(개발 PC, 좁은 재탐색 기준) 약 4.5초/사이클 - 라즈베리파이는 이보다 느릴 수 있어 여유를 둠.
# 너무 짧으면(예: 0.5초) 매 사이클마다 "명령 끊김"으로 오인해서 멈췄다 움직였다를 반복하고,
# 너무 길면 위치를 완전히 놓쳐 전체 재탐색(최대 수십초)에 들어간 상황에서도 계속 순항하게 되어
# 위험함. 좁은 재탐색은 타고 넘어가되 전체 재탐색처럼 오래 끊기면 정지하도록 중간값으로 설정.
STALE_COMMAND_TIMEOUT_S = 12.0
# Path_Planning/Explore_Map 판단과 무관하게 Throttle 자체에서 강제 정지하는 최후 안전 거리.
# Path_Planning 쪽 EMERGENCY_STOP_DIST_MM(80mm)보다 더 작게 잡아서, 평소엔 Path_Planning이
# 먼저 정지 명령을 내리고 이건 "그마저 안 걸렸을 때"만 발동하는 최후 백업으로만 쓰이게 함.
# 이 값이 Path_Planning 쪽보다 크면 정상 경로 위에서도 이 체크가 먼저/계속 걸려서 회전도 안
# 되는(v=0) 교착상태에 빠질 수 있음 (Explore_Map 시뮬레이션에서 재현된 것과 동일한 유형의 버그).
SAFETY_STOP_DIST_MM = 60.0


def read_latest_speed_command():
    if not os.path.exists(SPEED_STATE_FILE):
        return None
    try:
        with open(SPEED_STATE_FILE) as f:
            data = json.load(f)
        return data["speed_percent"], data.get("min_obstacle_dist_mm"), data["timestamp"]
    except (json.JSONDecodeError, KeyError, OSError):
        return None


def resolve_target_speed(now):
    """speed_command.json 을 읽어서 목표 speed_percent(-100~100)를 결정 (오래됐거나 위험하면 0)."""
    cmd = read_latest_speed_command()
    if cmd is None or (now - cmd[2]) > STALE_COMMAND_TIMEOUT_S:
        return 0.0

    speed_percent, min_obstacle_dist_mm, _ = cmd
    if min_obstacle_dist_mm is not None and min_obstacle_dist_mm < SAFETY_STOP_DIST_MM:
        # Path_Planning이 뭐라고 하든 Throttle 자체 판단으로 강제 정지 (최후 안전 반사)
        return 0.0
    return max(-100.0, min(100.0, speed_percent))


def publish_throttle_output(speed_percent):
    """최종 speed_percent 를 THROTTLE_OUTPUT_FILE 에 원자적으로 기록 (Ras_output.py가 읽어감)"""
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp_path = THROTTLE_OUTPUT_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump({"speed_percent": speed_percent, "timestamp": time.time()}, f)
    os.replace(tmp_path, THROTTLE_OUTPUT_FILE)  # 원자적 교체: Ras_output이 쓰다 만 파일을 읽지 않도록 함


def main():
    current_speed = 0.0
    print("구동(Throttle) 계산 시작 (Ctrl+C 종료)")
    try:
        while True:
            target_speed = resolve_target_speed(time.time())

            # 가감속 램프: 목표값으로 한 번에 뛰지 않고 주기당 ACCEL_STEP_PERCENT 만큼만 변화
            if target_speed > current_speed:
                current_speed = min(target_speed, current_speed + ACCEL_STEP_PERCENT)
            else:
                current_speed = max(target_speed, current_speed - ACCEL_STEP_PERCENT)

            publish_throttle_output(current_speed)
            time.sleep(CONTROL_PERIOD_S)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        publish_throttle_output(0.0)  # 종료 시 정지로 마지막 기록


if __name__ == "__main__":
    main()
