#!/bin/bash
# install_requirements.sh
# =========================
# Testing/ 를 라즈베리파이에서 돌리는 데 필요한 패키지를 전부 설치.
# 사용법: ./install_requirements.sh
#
# requirements.txt(numpy/pandas/matplotlib/scipy/opencv-python/pyserial)는 pip로,
# RPi.GPIO/picamera2 는 하드웨어(GPIO, libcamera)에 바로 붙는 패키지라 apt로 설치합니다
# (picamera2는 pip보다 apt 설치가 공식적으로 권장되고, Raspberry Pi OS Bookworm 이후엔
# 이미 깔려있는 경우가 많습니다).
#
# 최신 라즈베리파이 OS(Bookworm 이후)는 시스템 파이썬을 "externally-managed"로 막아둬서
# 그냥 pip install 하면 오류가 납니다. 그래서 --break-system-packages 를 붙입니다
# (가상환경을 따로 쓰고 싶다면 이 스크립트 실행 전에 직접 venv를 만들고 activate 하세요).

set -e
cd "$(dirname "$0")"

echo "=== apt 패키지 목록 갱신 ==="
sudo apt update

echo "=== RPi.GPIO / picamera2 (apt) 설치 ==="
sudo apt install -y python3-rpi.gpio python3-picamera2

echo "=== requirements.txt (pip) 설치 ==="
pip3 install --break-system-packages -r requirements.txt

echo "완료."
