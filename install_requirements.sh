#!/bin/bash
# install_requirements.sh
# =========================
# Testing/ 를 (Ubuntu가 설치된) 라즈베리파이에서 돌리는 데 필요한 패키지를 전부 설치.
# 사용법: ./install_requirements.sh
#
# 원래 라즈베리파이 OS 기준으로 작성됐었는데, 실제로는 라즈베리파이에 Ubuntu를 설치해서
# 씁니다. 그래서 라즈베리파이 OS 전용 apt 패키지에 의존하던 부분은 정리했습니다:
#   - RPi.GPIO: 이제 이 레포 어디서도 쓰지 않습니다. 구동 모터는 STM32에 CAN으로 명령만
#     보내고(Drive/OutInterface/Ras_output.py), 라이다는 USB 어댑터가 회전 모터 구동까지
#     자체 처리해서(/dev/ttyUSB0) GPIO가 필요 없어졌습니다. 그래서 이 스크립트에서도 뺐습니다.
#   - picamera2: 카메라용으로 여전히 필요해서 apt로 설치합니다. Ubuntu 22.04+ for Raspberry Pi는
#     Canonical과 Raspberry Pi Ltd 협업으로 universe 저장소에 python3-picamera2를 올려뒀습니다
#     (Raspberry Pi OS 전용이 아닙니다).
#
# requirements.txt(numpy/pandas/matplotlib/scipy/opencv-python/pyserial/python-can)는 pip로 설치합니다.
#
# Ubuntu 23.04+ 도 Raspberry Pi OS Bookworm과 마찬가지로 시스템 파이썬을
# "externally-managed"로 막아둬서 그냥 pip install 하면 오류가 납니다. 그래서
# --break-system-packages 를 붙입니다 (가상환경을 따로 쓰고 싶다면 이 스크립트 실행 전에
# 직접 venv를 만들고 activate 하세요).

set -e
cd "$(dirname "$0")"

echo "=== apt 패키지 목록 갱신 ==="
sudo apt update

echo "=== universe 저장소 활성화 (picamera2가 여기 있음) ==="
sudo add-apt-repository -y universe
sudo apt update

echo "=== picamera2 (apt) 설치 ==="
sudo apt install -y python3-picamera2

echo "=== requirements.txt (pip) 설치 ==="
pip3 install --break-system-packages -r requirements.txt

echo "완료."
