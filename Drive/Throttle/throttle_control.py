"""
throttle_control.py
=====================
자율주행 차량의 실제 주행 속도(구동 모터)를 제어합니다.
Path_Planning 쪽에서 계산한 목표 속도(SPEED_STATE_FILE)를 읽어서 DC모터를 구동하는 역할만 담당하고,
"얼마나 빠르게 갈지/멈출지"는 상위(Path_Planning) 판단을 그대로 따릅니다.

Steer와 마찬가지로 아직 실제 구동 모터/드라이버 사양이 정해지지 않아서
H-브릿지(방향 2핀 + PWM 1핀) 구성을 가정한 대략적인 뼈대만 만들어뒀습니다.
모터가 정해지면 핀 번호, PWM 주파수, duty 범위를 다시 맞춰야 합니다.

[안전 반사 (Path_Planning과는 독립적인 최후 방어선)]
장애물을 피해서 "어디로 갈지"를 다시 계산하는 판단은 전적으로 Path_Planning의 몫이고,
여기서는 그 결과(speed_percent)를 그대로 따른다. 다만 Path_Planning이 같이 실어 보내는
min_obstacle_dist_mm 값이 SAFETY_STOP_DIST_MM보다 가까우면, speed_percent가 뭐라고 되어
있든 상관없이 이 모듈이 자체적으로 강제 정지한다. Path_Planning 쪽 판단 로직에 버그가 있거나
응답이 늦어지는 경우를 대비한 하드웨어에 가장 가까운 마지막 안전망이다.
"""

import json
import os
import time

import RPi.GPIO as GPIO

# Testing/ 폴더를 통째로 옮겨도 항상 Testing/state 를 가리키도록 스크립트 위치 기준 상대경로로 계산
# (이 파일 위치: Testing/Drive/Throttle/throttle_control.py -> 두 단계 위가 Testing/)
TESTING_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STATE_DIR = os.path.join(TESTING_DIR, "state")
SPEED_STATE_FILE = os.path.join(STATE_DIR, "speed_command.json")

# ---- 구동 모터 핀 설정 (H-브릿지 가정, Steer와 겹치지 않는 핀 사용) ----
# TODO: 실제 모터 드라이버 스펙 확정되면 배선 확인 후 수정
PIN_IN1 = 23
PIN_IN2 = 24
PIN_PWM = 25
PWM_FREQ_HZ = 1000

MAX_SPEED_DUTY_PERCENT = 70    # 최고 속도 duty (처음엔 낮게 시작해서 테스트하며 올리기)
MIN_MOVING_DUTY_PERCENT = 30   # 이 미만이면 모터가 안 굴러가는 경우가 많음 (모터마다 다름, 튜닝 필요)

ACCEL_STEP_PERCENT = 2.0       # 제어 주기당 duty 변화 제한폭 (급가속/급제동 방지)
CONTROL_PERIOD_S = 0.02        # 약 50Hz

# Path_Planning은 매 프레임 도는 Camera와 달리 사이클 자체가 느림(위치추정 포함).
# 실측(개발 PC, 좁은 재탐색 기준) 약 4.5초/사이클 - 라즈베리파이는 이보다 느릴 수 있어 여유를 둠.
# 너무 짧으면(예: 0.5초) 매 사이클마다 "명령 끊김"으로 오인해서 멈췄다 움직였다를 반복하고,
# 너무 길면 위치를 완전히 놓쳐 전체 재탐색(최대 수십초)에 들어간 상황에서도 계속 순항하게 되어
# 위험함. 좁은 재탐색은 타고 넘어가되 전체 재탐색처럼 오래 끊기면 정지하도록 중간값으로 설정.
STALE_COMMAND_TIMEOUT_S = 12.0
# Path_Planning/Explore_Map 판단과 무관하게 Drive 자체에서 강제 정지하는 최후 안전 거리.
# Path_Planning 쪽 EMERGENCY_STOP_DIST_MM(80mm)보다 더 작게 잡아서, 평소엔 Path_Planning이
# 먼저 정지 명령을 내리고 이건 "그마저 안 걸렸을 때"만 발동하는 최후 백업으로만 쓰이게 함.
# 이 값이 Path_Planning 쪽보다 크면 정상 경로 위에서도 이 체크가 먼저/계속 걸려서 회전도 안
# 되는(v=0) 교착상태에 빠질 수 있음 (Explore_Map 시뮬레이션에서 재현된 것과 동일한 유형의 버그).
SAFETY_STOP_DIST_MM = 60.0


def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_IN1, GPIO.OUT)
    GPIO.setup(PIN_IN2, GPIO.OUT)
    GPIO.setup(PIN_PWM, GPIO.OUT)
    pwm = GPIO.PWM(PIN_PWM, PWM_FREQ_HZ)
    pwm.start(0)
    return pwm


def speed_percent_to_duty(speed_percent):
    """speed_percent: -100(최대 후진) ~ 0(정지) ~ 100(최대 전진)"""
    speed_percent = max(-100.0, min(100.0, speed_percent))
    if abs(speed_percent) < 1e-3:
        return "stop", 0.0
    ratio = abs(speed_percent) / 100.0
    duty = MIN_MOVING_DUTY_PERCENT + ratio * (MAX_SPEED_DUTY_PERCENT - MIN_MOVING_DUTY_PERCENT)
    direction = "forward" if speed_percent > 0 else "backward"
    return direction, duty


def drive_motor(pwm, direction, duty_percent):
    if direction == "stop":
        GPIO.output(PIN_IN1, GPIO.LOW)
        GPIO.output(PIN_IN2, GPIO.LOW)
        pwm.ChangeDutyCycle(0)
    elif direction == "forward":
        GPIO.output(PIN_IN1, GPIO.HIGH)
        GPIO.output(PIN_IN2, GPIO.LOW)
        pwm.ChangeDutyCycle(duty_percent)
    elif direction == "backward":
        GPIO.output(PIN_IN1, GPIO.LOW)
        GPIO.output(PIN_IN2, GPIO.HIGH)
        pwm.ChangeDutyCycle(duty_percent)


def read_latest_speed_command():
    if not os.path.exists(SPEED_STATE_FILE):
        return None
    try:
        with open(SPEED_STATE_FILE) as f:
            data = json.load(f)
        return data["speed_percent"], data.get("min_obstacle_dist_mm"), data["timestamp"]
    except (json.JSONDecodeError, KeyError, OSError):
        return None


def main():
    pwm = setup_gpio()
    current_duty = 0.0
    current_direction = "stop"
    print("구동(Throttle) 제어 시작 (Ctrl+C 종료)")
    try:
        while True:
            cmd = read_latest_speed_command()
            if cmd is None or (time.time() - cmd[2]) > STALE_COMMAND_TIMEOUT_S:
                target_direction, target_duty = "stop", 0.0
            else:
                speed_percent, min_obstacle_dist_mm, _ = cmd
                if min_obstacle_dist_mm is not None and min_obstacle_dist_mm < SAFETY_STOP_DIST_MM:
                    # Path_Planning이 뭐라고 하든 Drive 자체 판단으로 강제 정지 (최후 안전 반사)
                    target_direction, target_duty = "stop", 0.0
                else:
                    target_direction, target_duty = speed_percent_to_duty(speed_percent)

            # 방향이 바뀌면 먼저 duty를 0까지 낮춘 뒤에 반대 방향으로 (모터/드라이버 보호)
            if target_direction != current_direction and current_duty > 0:
                current_duty = max(0.0, current_duty - ACCEL_STEP_PERCENT)
            elif target_duty > current_duty:
                current_duty = min(target_duty, current_duty + ACCEL_STEP_PERCENT)
            else:
                current_duty = max(target_duty, current_duty - ACCEL_STEP_PERCENT)

            current_direction = "stop" if current_duty <= 0.0 else target_direction
            drive_motor(pwm, current_direction, current_duty)
            time.sleep(CONTROL_PERIOD_S)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        drive_motor(pwm, "stop", 0)
        pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
