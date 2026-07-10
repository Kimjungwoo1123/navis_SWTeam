"""
steer_control.py
==================
Camera(차선 인식)와 Path_Planning(라이다 기반 Pure Pursuit) 둘 다 각자 조향각을 계산해서
서로 다른 파일(camera_steering.json / lidar_steering.json)에 기록해두면, 이 스크립트가
그 둘을 비교해서 "최종적으로 어느 쪽을 따를지"를 정합니다.

[두 조향각을 섞는 기준]
Camera는 프레임마다(개발 PC 실측 ~60ms/frame, 라즈베리파이는 더 느릴 수 있음) 계산되고,
Path_Planning은 위치추정(ICP)까지 포함해서 사이클당 훨씬 오래 걸립니다(개발 PC 실측 좁은
재탐색 기준으로도 ~4.5초/cycle - 라즈베리파이는 이보다 더 걸릴 가능성이 큽니다). 그래서:
  - 라이다 쪽 값이 없거나 너무 오래돼서(LIDAR_STALE_TIMEOUT_S) 못 미더우면 -> 카메라만 사용
    (평소 대부분의 순간이 이 경우입니다 - 카메라가 기본/주 조향원)
  - 카메라 값이 없거나 오래됐는데 라이다 값은 최신이면 -> 라이다 값을 대신 사용 (카메라 고장 대비)
  - 둘 다 최신이면 -> 두 값의 차이가 STEERING_AGREEMENT_TOLERANCE_DEG 이내로 서로 동의할 때만
    라이다(경로 인지) 값을 채택하고, 차이가 크면(서로 의견이 갈리면) 더 빠르고 반응성 좋은
    카메라 값을 그대로 사용합니다.
  - 둘 다 없거나 오래되면 -> 안전하게 정지(중립).

[출력]
실제 모터 구동은 이 스크립트가 아니라 STM32가 담당하고, 라즈베리파이는 CAN으로 명령만
넘깁니다(Drive/OutInterface/Ras_output.py). 그래서 이 스크립트는 GPIO를 전혀 건드리지 않고,
매 주기 계산한 최종 조향각을 STEER_OUTPUT_FILE(steer_output.json)에 기록만 합니다.
Ras_output.py가 이 값을 읽어서 STM32 CAN 프로토콜(각도 30~150, 90=정면)로 변환해 전송합니다.
"""

import json
import os
import time

# Testing/ 폴더를 통째로 옮겨도 항상 Testing/state 를 가리키도록 스크립트 위치 기준 상대경로로 계산
# (이 파일 위치: Testing/Drive/Steer/steer_control.py -> 두 단계 위가 Testing/)
TESTING_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STATE_DIR = os.path.join(TESTING_DIR, "state")
CAMERA_STEERING_FILE = os.path.join(STATE_DIR, "camera_steering.json")
LIDAR_STEERING_FILE = os.path.join(STATE_DIR, "lidar_steering.json")
STEER_OUTPUT_FILE = os.path.join(STATE_DIR, "steer_output.json")

MAX_STEERING_DEG = 30.0    # Camera/Path_Planning 쪽과 동일하게 맞춰야 함

# Camera는 빠르게(프레임마다) 갱신되므로 짧게, Path_Planning은 사이클이 훨씬 길어서(수 초~수십 초,
# "Path_Planning 사이클타임 벤치마크" 결과 기준) 훨씬 길게 잡음. 실제 라즈베리파이에서 측정한
# 값으로 다시 튜닝 필요.
CAMERA_STALE_TIMEOUT_S = 0.5
LIDAR_STALE_TIMEOUT_S = 15.0
STEERING_AGREEMENT_TOLERANCE_DEG = 5.0  # 이 이내로 서로 동의할 때만 라이다(경로) 값을 채택

CONTROL_PERIOD_S = 0.02        # 약 50Hz


def read_angle_file(path):
    """{"angle_deg":..., "timestamp":...} 파일을 읽음. 없거나 깨졌으면 None."""
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        return data["angle_deg"], data["timestamp"]
    except (json.JSONDecodeError, KeyError, OSError):
        return None


def resolve_steering_angle(now):
    """
    camera_steering.json / lidar_steering.json 을 읽어서 최종 조향각을 결정.
    반환: (angle_deg 또는 None, 선택된 소스 문자열 - 로그/디버깅용)
    None이면 둘 다 없거나 오래돼서 안전하게 정지(중립)해야 함을 뜻함.
    """
    cam = read_angle_file(CAMERA_STEERING_FILE)
    lidar = read_angle_file(LIDAR_STEERING_FILE)

    cam_fresh = cam is not None and (now - cam[1]) <= CAMERA_STALE_TIMEOUT_S
    lidar_fresh = lidar is not None and (now - lidar[1]) <= LIDAR_STALE_TIMEOUT_S

    if cam_fresh and lidar_fresh:
        diff = abs(cam[0] - lidar[0])
        if diff <= STEERING_AGREEMENT_TOLERANCE_DEG:
            return lidar[0], "lidar(agree)"
        return cam[0], "camera(disagree)"
    if cam_fresh:
        return cam[0], "camera(lidar stale)"
    if lidar_fresh:
        return lidar[0], "lidar(camera stale)"
    return None, "none(both stale)"


def publish_steer_output(angle_deg):
    """최종 조향각을 STEER_OUTPUT_FILE 에 원자적으로 기록 (Ras_output.py가 읽어감)"""
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp_path = STEER_OUTPUT_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump({"angle_deg": angle_deg, "timestamp": time.time()}, f)
    os.replace(tmp_path, STEER_OUTPUT_FILE)  # 원자적 교체: Ras_output이 쓰다 만 파일을 읽지 않도록 함


def main():
    print("조향(Steer) 계산 시작 (Ctrl+C 종료)")
    try:
        while True:
            angle_deg, source = resolve_steering_angle(time.time())
            if angle_deg is None:
                angle_deg = 0.0  # 중립(정면) - 안전
            angle_deg = max(-MAX_STEERING_DEG, min(MAX_STEERING_DEG, angle_deg))
            publish_steer_output(angle_deg)
            time.sleep(CONTROL_PERIOD_S)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        publish_steer_output(0.0)  # 종료 시 중립으로 마지막 기록


if __name__ == "__main__":
    main()
