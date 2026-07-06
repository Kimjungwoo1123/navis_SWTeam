import csv
import os
import time
import struct

import serial
import RPi.GPIO as GPIO

# PWM 설정 (CTRL 핀 - GPIO12)
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
pwm = GPIO.PWM(12, 50000)  # 50kHz PWM
pwm.start(50)  # 듀티사이클 50%

# 시리얼 포트 설정
ser = serial.Serial(
    port='/dev/ttyS0',  # 안되면 /dev/ttyAMA0 으로 바꿔봐
    baudrate=230400,
    timeout=1
)

OUT_CSV = f"lidar_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
CSV_HEADER = ["system_time", "lidar_timestamp_ms", "angle_deg", "distance_mm", "intensity"]

print("LD06 LiDAR 시작...")
print(f"저장 파일: {OUT_CSV}  (Ctrl+C 로 종료하면 저장 완료)")


def parse_packet(data):
    # LD06 패킷 파싱
    # 헤더: 0x54, 길이: 0x2C
    if data[0] != 0x54 or data[1] != 0x2C:
        return None

    speed = struct.unpack('<H', data[2:4])[0] / 100.0  # deg/s
    start_angle = struct.unpack('<H', data[4:6])[0] / 100.0  # deg

    points = []
    for i in range(12):
        offset = 6 + i * 3
        distance = struct.unpack('<H', data[offset:offset+2])[0]  # mm
        intensity = data[offset+2]
        points.append((distance, intensity))

    end_angle = struct.unpack('<H', data[42:44])[0] / 100.0  # deg
    timestamp = struct.unpack('<H', data[44:46])[0]  # ms

    return {
        'speed': speed,
        'start_angle': start_angle,
        'end_angle': end_angle,
        'points': points,
        'timestamp': timestamp
    }


def point_angles(start_angle, end_angle, n_points):
    """패킷 하나에 담긴 n_points개 점의 각 각도를 start~end 사이에서 보간.
    LD06은 0도 부근에서 각도가 랩어라운드(예: 350 -> 10) 하므로 그만큼 보정."""
    diff = end_angle - start_angle
    if diff < 0:
        diff += 360.0
    step = diff / (n_points - 1) if n_points > 1 else 0.0
    return [(start_angle + step * i) % 360.0 for i in range(n_points)]


try:
    with open(OUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)

        buffer = bytearray()
        while True:
            raw = ser.read(47)  # LD06 패킷 크기 47바이트
            if len(raw) == 47:
                result = parse_packet(raw)
                if result:
                    now = time.time()
                    angles = point_angles(result['start_angle'], result['end_angle'],
                                           len(result['points']))
                    for angle_deg, (distance, intensity) in zip(angles, result['points']):
                        writer.writerow([f"{now:.6f}", result['timestamp'],
                                          f"{angle_deg:.2f}", distance, intensity])

                    print(f"속도: {result['speed']:.1f} deg/s | "
                          f"각도: {result['start_angle']:.1f}~{result['end_angle']:.1f}° | "
                          f"거리(첫번째): {result['points'][0][0]} mm")

except KeyboardInterrupt:
    print(f"\n종료 - 저장됨: {os.path.abspath(OUT_CSV)}")
    pwm.stop()
    GPIO.cleanup()
    ser.close()
