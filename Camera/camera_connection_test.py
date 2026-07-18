"""
camera_connection_test.py
==========================
카메라 하드웨어 연결/캡처가 정상인지만 확인하는 순수 스모크 테스트.
라이다, 조향 제어 등 다른 하드웨어는 전혀 건드리지 않고 카메라 한 대만 연결돼 있으면
라즈베리파이(Picamera2 CSI 카메라)와 젯슨 오린(nvarguscamerasrc CSI 카메라, 또는
V4L2 USB 카메라) 양쪽에서 수정 없이 그대로 실행할 수 있습니다.

[확인 항목]
1. 실행 중인 보드가 라즈베리파이인지 젯슨(테그라)인지 자동 판별
2. 판별된 보드에 맞는 방식으로 카메라를 염 (실패하면 일반 V4L2 USB 카메라로 폴백)
3. 지정한 프레임 수만큼 실제로 캡처해서 평균 FPS 측정
4. 캡처된 프레임이 새까맣거나(렌즈캡 등) 신호가 없는 상태는 아닌지 밝기 표준편차로 확인
5. 마지막 프레임 1장을 이 스크립트 옆에 저장 (디스플레이 없는 헤드리스 환경에서도
   scp 등으로 받아서 눈으로 확인 가능하도록)

lane_center_detection.py와 달리 차선 인식 로직은 포함하지 않음 - 이 스크립트는
"카메라가 물리적으로 잘 연결돼서 정상적인 영상을 주는가"만 책임짐.
"""

import argparse
import os
import sys
import time

import numpy as np
import cv2

try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SNAPSHOT_PATH = os.path.join(SCRIPT_DIR, "camera_test_snapshot.jpg")

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DEFAULT_NUM_FRAMES = 60

# 새까만/깨진 프레임(렌즈캡, 신호없음 등) 판별 기준 - 정상 영상의 밝기 표준편차는 보통 이보다 훨씬 큼
MIN_BRIGHTNESS_STD = 3.0


def detect_board():
    """실행 중인 보드가 라즈베리파이인지 젯슨(테그라)인지 판별. 둘 다 아니면 "unknown"."""
    if os.path.exists("/etc/nv_tegra_release"):
        return "jetson"
    model_path = "/proc/device-tree/model"
    if os.path.exists(model_path):
        try:
            with open(model_path, "rb") as f:
                model = f.read().decode(errors="ignore")
        except OSError:
            model = ""
        if "Raspberry Pi" in model:
            return "raspberrypi"
        if "NVIDIA Jetson" in model or "Orin" in model:
            return "jetson"
    return "unknown"


def jetson_csi_gstreamer_pipeline(width=FRAME_WIDTH, height=FRAME_HEIGHT, framerate=30):
    """젯슨 오린 CSI 카메라(nvarguscamerasrc)용 GStreamer 파이프라인 문자열.
    ISP를 거쳐 BGR로 변환해서 OpenCV가 바로 받을 수 있게 함."""
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"framerate={framerate}/1, format=NV12 ! "
        f"nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
    )


def open_camera(board):
    """보드에 맞는 방식으로 카메라를 열어서 (핸들, backend 설명 문자열)을 반환.
    다 실패하면 RuntimeError."""
    if board == "raspberrypi" and HAS_PICAMERA2:
        cam = Picamera2()
        config = cam.create_preview_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"})
        cam.configure(config)
        cam.start()
        return cam, "picamera2 (Raspberry Pi CSI)"

    if board == "jetson":
        pipeline = jetson_csi_gstreamer_pipeline()
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            return cap, "GStreamer nvarguscamerasrc (Jetson CSI)"
        cap.release()
        print("  [안내] nvarguscamerasrc로 CSI 카메라를 열지 못했습니다 - USB 카메라(V4L2)로 시도합니다.")

    # 라즈베리파이인데 picamera2가 없거나(USB 카메라를 쓰는 경우), 젯슨인데 CSI가 아니거나,
    # 보드를 못 알아냈을 때 공통 폴백 - 대부분의 USB 웹캠은 이걸로 잡힘
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다 (CSI/USB 모두 실패).")
    return cap, "V4L2 (/dev/video0)"


def read_frame(cam):
    if HAS_PICAMERA2 and isinstance(cam, Picamera2):
        frame = cam.capture_array()
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    ok, frame = cam.read()
    if not ok:
        raise RuntimeError("프레임을 읽지 못했습니다.")
    return frame


def close_camera(cam):
    if HAS_PICAMERA2 and isinstance(cam, Picamera2):
        cam.stop()
    else:
        cam.release()


def run_test(num_frames):
    board = detect_board()
    print(f"보드 판별 결과: {board}")

    cam, backend = open_camera(board)
    print(f"카메라 열기 성공 - backend: {backend}")

    frames_captured = 0
    brightness_stds = []
    last_frame = None
    t0 = time.time()
    try:
        for _ in range(num_frames):
            frame = read_frame(cam)
            frames_captured += 1
            last_frame = frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            brightness_stds.append(float(np.std(gray)))
    finally:
        elapsed_s = time.time() - t0
        close_camera(cam)

    if frames_captured == 0:
        print("실패: 프레임을 한 장도 받지 못했습니다.")
        return False

    fps = frames_captured / elapsed_s if elapsed_s > 0 else 0.0
    avg_std = float(np.mean(brightness_stds))
    h, w = last_frame.shape[:2]

    print(f"캡처한 프레임: {frames_captured}/{num_frames}, 소요시간: {elapsed_s:.1f}s, "
          f"평균 FPS: {fps:.1f}, 해상도: {w}x{h}")
    print(f"밝기 표준편차(평균): {avg_std:.1f} (기준: {MIN_BRIGHTNESS_STD} 이상이어야 정상 영상)")

    ok = True
    if avg_std < MIN_BRIGHTNESS_STD:
        print("경고: 프레임이 거의 단색(새까맣거나 흰색)입니다 - 렌즈캡을 씌운 채이거나 "
              "센서가 신호를 못 받고 있을 수 있습니다.")
        ok = False

    cv2.imwrite(SNAPSHOT_PATH, last_frame)
    print(f"마지막 프레임 저장: {SNAPSHOT_PATH} (디스플레이 없으면 이 파일을 받아서 눈으로 확인)")

    print("결과:", "정상" if ok else "이상 있음")
    return ok


def main():
    parser = argparse.ArgumentParser(description="카메라 연결/캡처 스모크 테스트 (라즈베리파이/젯슨 오린 겸용)")
    parser.add_argument("--frames", type=int, default=DEFAULT_NUM_FRAMES,
                         help=f"테스트로 캡처할 프레임 수 (기본 {DEFAULT_NUM_FRAMES})")
    args = parser.parse_args()

    try:
        ok = run_test(args.frames)
    except Exception as e:
        print(f"실패: {e}")
        sys.exit(1)

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
