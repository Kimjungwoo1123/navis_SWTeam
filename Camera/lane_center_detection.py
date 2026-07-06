"""
lane_center_detection.py
==========================
라즈베리파이 카메라로 도로 영상을 받아서 차선을 인식하고,
화면 중앙(=차량 중앙) 대비 차선 중앙이 얼마나 벗어났는지로 조향각(steering angle)을 계산합니다.

[동작 방식]
1. 프레임 하단 영역(ROI)만 잘라서 사용 (하늘/먼 배경 제외)
2. Canny 엣지 + Hough 직선 검출로 선분들을 찾음
3. 기울기 부호로 좌측 차선 / 우측 차선 후보를 분리
4. 화면 하단 기준 좌/우 차선의 x좌표 평균으로 "차선 중앙"을 추정
5. 차선 중앙 - 화면 중앙 = 오프셋(px) -> 비례 게인(STEERING_GAIN) 곱해서 조향각(deg)으로 변환
6. 계산된 조향각을 CAMERA_STEERING_FILE 에 기록

Path_Planning(라이다 기반 Pure Pursuit)도 별도로 lidar_steering.json 에 자기 조향각을
기록하는데, Drive/Steer 가 두 값을 비교해서 최종 조향을 정합니다 (카메라가 훨씬 빠르게
갱신되므로 기본은 카메라, 라이다와 카메라 판단이 오차범위 안에서 일치할 때만 라이다 값을
우선 - 자세한 기준은 Steer/steer_control.py 참고). 즉 이 파일은 "카메라 혼자만의 의견"만
책임지고 최종 결정에는 관여하지 않습니다.

카메라/렌즈, 실제 차선 폭, 조향 기구 특성에 따라 STEERING_GAIN, MAX_STEERING_DEG,
ROI 영역 등은 실측하면서 튜닝이 필요합니다. 참고로 개발 PC(8코어)에서 640x480 프레임
처리에 평균 약 60ms(~16Hz)가 걸렸는데, 라즈베리파이에서는 이보다 느릴 가능성이 큽니다 -
LOOP_PERIOD_S는 "처리 후 추가로 쉬는 시간"일 뿐 실제 주기를 보장하지는 않습니다.
"""

import json
import os
import time

import numpy as np
import cv2

try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False

# Testing/ 폴더를 통째로 옮겨도 항상 Testing/state 를 가리키도록 스크립트 위치 기준 상대경로로 계산
# (이 파일 위치: Testing/Camera/lane_center_detection.py -> 한 단계 위가 Testing/)
TESTING_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATE_DIR = os.path.join(TESTING_DIR, "state")
CAMERA_STEERING_FILE = os.path.join(STATE_DIR, "camera_steering.json")

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
ROI_TOP_RATIO = 0.55  # 화면 아래 45%만 차선 인식에 사용 (도로 바닥 위주)

MAX_STEERING_DEG = 30.0  # 최대 조향각(좌우). 실제 조향 기구 한계에 맞춰 조정
STEERING_GAIN = 0.08     # 픽셀 오프셋 -> 각도 변환 비례 게인 (튜닝 필요)

CANNY_LOW, CANNY_HIGH = 50, 150
HOUGH_THRESHOLD = 20
HOUGH_MIN_LINE_LEN = 20
HOUGH_MAX_LINE_GAP = 100

LOOP_PERIOD_S = 0.05  # 약 20Hz


def open_camera():
    if HAS_PICAMERA2:
        cam = Picamera2()
        config = cam.create_preview_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"})
        cam.configure(config)
        cam.start()
        return cam
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다.")
    return cap


def read_frame(cam):
    if HAS_PICAMERA2:
        return cam.capture_array()
    ok, frame = cam.read()
    if not ok:
        raise RuntimeError("프레임을 읽지 못했습니다.")
    return frame


def region_of_interest(edges):
    h, w = edges.shape
    mask = np.zeros_like(edges)
    roi_top = int(h * ROI_TOP_RATIO)
    polygon = np.array([[(0, h), (0, roi_top), (w, roi_top), (w, h)]], dtype=np.int32)
    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(edges, mask)


def detect_lane_lines(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
    roi = region_of_interest(edges)
    return cv2.HoughLinesP(roi, 1, np.pi / 180, HOUGH_THRESHOLD,
                            minLineLength=HOUGH_MIN_LINE_LEN, maxLineGap=HOUGH_MAX_LINE_GAP)


def split_left_right(lines, frame_width):
    """기울기 부호로 좌/우 차선 후보를 분리 (이미지 좌표계: 아래로 갈수록 y 증가)"""
    left_pts, right_pts = [], []
    if lines is None:
        return left_pts, right_pts
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 == x1:
            continue
        slope = (y2 - y1) / (x2 - x1)
        if abs(slope) < 0.3:  # 거의 수평인 노이즈 라인 제거
            continue
        if slope < 0 and max(x1, x2) < frame_width * 0.6:
            left_pts.append((x1, y1, x2, y2))
        elif slope > 0 and min(x1, x2) > frame_width * 0.4:
            right_pts.append((x1, y1, x2, y2))
    return left_pts, right_pts


def average_x_at_bottom(pts, y_bottom):
    """선분들을 y_bottom 지점까지 연장했을 때의 x좌표 평균"""
    if not pts:
        return None
    xs = []
    for x1, y1, x2, y2 in pts:
        if y2 == y1:
            continue
        slope = (x2 - x1) / (y2 - y1)
        xs.append(x1 + slope * (y_bottom - y1))
    return float(np.mean(xs)) if xs else None


def compute_steering_angle(frame):
    """
    프레임에서 차선 중앙 대비 오프셋으로 조향각(deg)을 계산.
    양수: 우측 조향, 음수: 좌측 조향. 차선을 하나도 못 찾으면 None.
    """
    h, w = frame.shape[:2]
    lines = detect_lane_lines(frame)
    left_pts, right_pts = split_left_right(lines, w)

    left_x = average_x_at_bottom(left_pts, h)
    right_x = average_x_at_bottom(right_pts, h)

    if left_x is not None and right_x is not None:
        lane_center_x = (left_x + right_x) / 2.0
    elif left_x is not None:
        lane_center_x = left_x + w * 0.25  # 한쪽 차선만 보이면 차선폭의 절반만큼 대략 보정
    elif right_x is not None:
        lane_center_x = right_x - w * 0.25
    else:
        return None

    offset_px = lane_center_x - (w / 2.0)
    angle = offset_px * STEERING_GAIN
    return max(-MAX_STEERING_DEG, min(MAX_STEERING_DEG, angle))


def publish_steering_angle(angle_deg):
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp_path = CAMERA_STEERING_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump({"angle_deg": angle_deg, "timestamp": time.time()}, f)
    os.replace(tmp_path, CAMERA_STEERING_FILE)  # 원자적 교체: Steer가 쓰다 만 파일을 읽지 않도록 함


def main():
    cam = open_camera()
    last_angle = 0.0
    print("차선 인식 + 조향각 계산 시작 (Ctrl+C 종료)")
    try:
        while True:
            frame = read_frame(cam)
            angle = compute_steering_angle(frame)
            if angle is None:
                angle = last_angle  # 차선을 순간적으로 놓치면 직전 각도 유지 (급조향 방지)
            else:
                last_angle = angle
            publish_steering_angle(angle)
            print(f"steering angle: {angle:+.1f} deg")
            time.sleep(LOOP_PERIOD_S)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        if HAS_PICAMERA2:
            cam.stop()
        else:
            cam.release()


if __name__ == "__main__":
    main()
