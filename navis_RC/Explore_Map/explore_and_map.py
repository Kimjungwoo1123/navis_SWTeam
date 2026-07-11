"""
explore_and_map.py
=====================
Build_Map(사람이 위치를 옮겨가며 스캔 찍기)를 대신해서, 사용자가 초기 위치에만 놓아주면
차가 혼자 돌아다니며 라이다로 지도를 만들고(자율 SLAM), 더 갈 곳이 없어지면 자동으로
초기 위치(원점)까지 돌아오는 스크립트입니다.

[동작 원리]
1. 최초 위치를 map 좌표계의 원점(0,0,0)으로 고정 (build_map.py의 pos1=원점 관례와 동일)
2. 매 사이클: 라이다 한 바퀴 스캔 -> 지금까지 쌓은 점군에 ICP로 대조해서 현재 위치 추정
   (Path_Planning/localize_and_plan.py 와 동일한 warm-start 방식: 직전 위치 근방만 좁게
   재탐색하고, 오차가 크면 더 넓은 범위로 재탐색)
3. 이번 스캔으로 3상태 grid(미탐색 UNKNOWN=-1 / 빈공간 FREE=0 / 장애물 OCCUPIED=1)를 갱신
   (raytrace: 로봇~장애물 사이는 FREE, 장애물 지점은 OCCUPIED)
4. "빈공간이면서 미탐색 영역과 맞닿은 셀"(프론티어)을 찾아서 가장 가깝고 갈 수 있는 곳으로 A*
   경로를 잡고 Pure Pursuit로 조향각 계산 -> Drive(Steer/Throttle)에 명령 발행
5. 탐색 종료 조건 (둘 중 하나라도 해당하면 종료):
   a) 더 이상 갈 프론티어가 없거나(전부 탐색 완료), 남은 프론티어가 전부 경로가 막혀있을 때
   b) 현재 위치에서 전/후/좌/우 네 방향이 전부 확인된 장애물(OCCUPIED)로 막혀 물리적으로
      더 이상 움직일 수 없을 때 (is_boxed_in) - 이게 없으면 A*가 매 사이클 실패만 반복하며
      사이클을 낭비하게 됨
6. 종료되면 지금까지 쌓은 지도를 build_map.py와 같은 형식(map_points.csv, map_occupancy_grid.npy,
   map_grid_meta.csv)으로 저장 -> 그 지도로 원점(0,0)까지 A*+Pure Pursuit로 복귀
   (원점 RETURN_HOME_TOLERANCE_MM 이내로 들어오면 복귀 완료 = 두 번째 프로그램 종료 조건)

[탐색 중 조향/속도 제한]
원래 처음 가보는 실내 공간이라는 이유로 조향/속도 모두 RC카 전체 한계(MAX_STEERING_DEG=30도,
speed_percent 100%)의 1/3만 쓰도록 제한해뒀었는데, 실기에서 힘이 부족해 조향이 다 안 꺾이고
속도도 정체(stall) 감지 기준을 채울 만큼 못 움직여 오탐(그냥 느린 것뿐인데 "걸렸다"고 판단)이
계속 나서, 둘 다 전체 출력(EXPLORE_MAX_STEERING_DEG=30도, CRUISE_SPEED_PERCENT_EXPLORE=100%)
까지 쓰도록 풀었습니다 (2026-07-11).

이 단계는 실내(차선 없음)라 Camera는 켤 필요가 없습니다 - Drive/Steer는 camera_steering.json이
없으면(또는 오래되면) 자동으로 lidar_steering.json만 보고 동작하도록 이미 만들어져 있어서
그대로 재사용됩니다. Drive(Steer/Throttle)는 이 스크립트를 실행하는 동안에도 평소처럼
Testing/state/ 의 speed_command.json, lidar_steering.json 을 읽는 방식 그대로입니다.

[실시간 시각화]
기본적으로 탐색/복귀 중 점유 격자(장애물/빈공간/미탐색), 로봇이 지나온 경로, 현재 위치·방향,
목표(프론티어/원점) 지점을 matplotlib 창으로 매 사이클 갱신해서 보여줍니다(LiveMapView).
디스플레이가 없는 SSH-only 환경이면 인터랙티브 백엔드 전환에 실패해 자동으로 꺼지고,
--no_live_view 로 직접 끌 수도 있습니다.

[한계]
- 벽만 있는 빈 공간처럼 특징이 적으면 ICP가 조금씩 틀어질 수 있고(누적 보정/loop closure 없음),
  다녀본 적 없는 큰 공간에서는 GRID_HALF_SIZE_MM 로 잡아둔 고정 크기 grid 밖으로 못 나갑니다.
  지금 방 크기(수 미터) 기준으로는 여유 있게 잡아뒀습니다.

[사용법]
    python3 explore_and_map.py --out ./map_output
    python3 explore_and_map.py --out ./map_output --no_live_view   # 실시간 창 없이(헤드리스)
"""

import os
import csv
import json
import time
import argparse
import heapq
import struct
import numpy as np
import pandas as pd
import matplotlib
try:
    matplotlib.use("TkAgg")   # 실시간 시각화용 인터랙티브 백엔드
except Exception:
    matplotlib.use("Agg")     # 디스플레이 없는 환경(헤드리스 SSH 등) - 최종 map_result.png 저장만 가능
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.ndimage import binary_dilation, label

# ----------------------------
# 상수
# ----------------------------
ICP_MAX_ITER = 40
ICP_TOLERANCE = 1e-5
ICP_MAX_CORRESPONDENCE_DIST = 300.0  # mm

MIN_RANGE_MM = 150
MAX_RANGE_MM = 8000

GRID_RESOLUTION_MM = 50
GRID_HALF_SIZE_MM = 8000  # 원점 기준 사방 8m (16m x 16m). 더 큰 공간이면 늘려야 함.

MAP_ICP_DOWNSAMPLE = 20000  # 누적 점군이 커져도 ICP 대상은 이 개수로 다운샘플

NARROW_POS_OFFSETS_MM = (-150.0, 0.0, 150.0)
NARROW_ANGLE_SPREAD_DEG = 15.0
RELOCALIZE_ERROR_THRESHOLD_MM = 150.0
WIDE_POS_OFFSETS_MM = (-450.0, -300.0, -150.0, 0.0, 150.0, 300.0, 450.0)
WIDE_ANGLE_STEP_DEG = 30

# wide 탐색도 prev_pose 근방(±450mm)만 보기 때문에, 정말로 크게 위치를 놓치면(예: 슬립,
# 순간적으로 심하게 가려진 스캔 등) narrow/wide 둘 다 영원히 못 돌아올 수 있다. IMU/GPS가
# 없어 라이다 스캔매칭이 유일한 위치 정보원이므로, wide까지 실패한 게 GLOBAL_RELOCALIZE_AFTER
# 만큼 연속되면 prev_pose 주변을 벗어나 "지금까지 쌓은 맵 전체"를 다시 뒤지는 전역 탐색으로
# 넘어간다 (localize()의 기본 5x5 격자 자동 생성 재사용).
GLOBAL_RELOCALIZE_AFTER_CONSECUTIVE_LOST = 3
GLOBAL_ANGLE_STEP_DEG = 45
# 전역 탐색까지 다 해봐도 이보다 오차가 크면 "진짜로 위치를 완전히 놓쳤다"고 보고 정지한다.
LOCALIZATION_LOST_ERROR_THRESHOLD_MM = 250.0

INFLATE_CELLS = 2
FRONTIER_MIN_CLUSTER_CELLS = 3   # 이보다 작은 프론티어 뭉치는 노이즈로 보고 무시
MAX_EXPLORE_CYCLES = 300          # 무한루프 방지용 안전장치

WHEELBASE_MM = 150.0     # TODO: 실제 차량 축간거리로 교체
LOOKAHEAD_MM = 400.0
MAX_STEERING_DEG = 30.0      # RC카 전체 조향 한계 (Camera/Path_Planning/Drive와 동일)
FULL_SPEED_PERCENT = 100.0   # RC카 전체 속도 한계 (STM32 CAN speed_percent 전체 범위)

# 탐색 단계는 원래 처음 가보는 실내 공간이라는 이유로 조향/속도 모두 RC카 전체 한계의 1/3로
# 제한했었는데, 실기에서 그 정도로는 힘이 부족해서(조향도 서보 토크가 부족해 완전히 안 꺾이고,
# 속도는 apply_min_moving_floor의 데드존(60%) 보정을 거쳐도 정체(stall) 감지 기준
# (STALL_DISPLACEMENT_MM/STALL_CHECK_WINDOW_S)을 채울 만큼도 못 움직여서 "그냥 느리게 가고
# 있을 뿐"인데 "걸려서 멈췄다"고 오판, 후진이 자꾸 발동함) 둘 다 전체 출력까지 풀었다
# (2026-07-11, 실기 피드백 반영). 안전을 원하면 다시 낮춰서 튜닝할 것.
EXPLORE_MAX_STEERING_DEG = MAX_STEERING_DEG          # 탐색도 전체 조향각 사용 (1/3 제한 해제)
CRUISE_SPEED_PERCENT_EXPLORE = FULL_SPEED_PERCENT    # 탐색도 전체 출력 사용 (1/3 속도제한 해제)

# 실기 캘리브레이션(2026-07-11, cansend로 직접 STM32에 speed만 바꿔가며 실측):
# speed_percent가 이 미만이면 정지 마찰을 못 이겨서 바퀴가 실제로 안 구른다
# (50=무반응, 60=반응 확인). 그래서 0(완전정지)이 아닌 이상 실제로 명령을 보낼 땐
# 항상 이 값 이상으로 올려서 보낸다 - 안 그러면 CRUISE_SPEED_PERCENT_EXPLORE(약 33%)나
# 곡률 감속(curvature_speed_factor, 최대 0.5배)이 적용된 값이 데드존 밑으로 떨어져서
# "계산상으론 전진 중"인데 실제로는 제자리에 멈춰있는 상태가 된다 - Explore_Map이
# 지금까지 안 움직인 것처럼 보였던 것도 이 문제였음. 모터/배터리가 바뀌면 다시 실측 필요.
MIN_MOVING_SPEED_PERCENT = 60.0


def apply_min_moving_floor(speed_percent):
    """0(정지)이면 그대로 두고, 그 외(전진 의도)엔 MIN_MOVING_SPEED_PERCENT 밑으로 내려가지
    않게 올려서 반환한다 - 데드존 이하 값을 보내서 모터가 안 움직이는 상태를 방지."""
    if speed_percent <= 0.0:
        return 0.0
    return max(MIN_MOVING_SPEED_PERCENT, min(speed_percent, FULL_SPEED_PERCENT))


# ----------------------------
# 정체(stall) 감지 + 후진 탈출
# 실기 로그에서 실제로 재현된 문제: 라이다 스캔 평면보다 낮은 바닥 장애물(문턱, 케이블 등)에
# 걸리면 lidar는 계속 "앞이 비어있다"고 보고해서 계속 전진 명령을 내리는데, 바퀴는 그 자리에서
# 헛돌기만 하고 차는 전혀 못 움직인다 - 장애물 회피(min_obstacle_dist_mm)로는 절대 못 잡아내는
# 사각지대. 그래서 "명령은 계속 나가는데 실제 위치가 안 바뀐다"는 것 자체를 별도로 감시한다.
# ----------------------------
# 2.0s로 처음 잡았다가 실기에서 정상 주행 중에도 오탐이 나서 5.0s로 늘림(2026-07-11) - 순항속도를
# 전체출력(100%)으로 올린 지금도, 가감속 램프(Throttle의 ACCEL_STEP_PERCENT)나 조향 중 회전 반경
# 때문에 짧은 시간 안에는 실제로 이동 거리가 작게 나올 수 있어 여유를 더 뒀다.
STALL_CHECK_WINDOW_S = 5.0        # 이 시간 동안의 이동량을 본다
STALL_DISPLACEMENT_MM = 50.0      # 이 시간 동안 이 거리도 못 움직였으면 "막혔다"고 판단
STALL_REVERSE_SPEED_PERCENT = -MIN_MOVING_SPEED_PERCENT  # 후진도 같은 데드존이 있다고 가정
# (전진 기준으로만 실측함 - 후진 데드존이 다르면 재조정 필요)
STALL_REVERSE_DURATION_S = 1.2    # 후진을 몇 초간 유지할지
STALL_AVOID_RADIUS_MM = 400.0     # 막혔던 지점 주변은 이후 프론티어 후보에서 계속 제외


class StallDetector:
    """직진/회전 명령을 계속 내는데도 실제 위치가 STALL_CHECK_WINDOW_S 동안
    STALL_DISPLACEMENT_MM 이상 못 움직이면 '막혔다'고 판단하는 헬퍼."""

    def __init__(self, window_s=STALL_CHECK_WINDOW_S, min_displacement_mm=STALL_DISPLACEMENT_MM):
        self.window_s = window_s
        self.min_displacement_mm = min_displacement_mm
        self.history = []  # [(t, x, y), ...] 오래된 것부터

    def reset(self):
        self.history.clear()

    def update(self, t, x, y):
        self.history.append((t, x, y))
        # window_s보다 오래된 기준점은 하나만 남기고 정리 (그게 비교 기준이 됨)
        while len(self.history) >= 2 and (t - self.history[1][0]) >= self.window_s:
            self.history.pop(0)

    def is_stalled(self, t):
        if not self.history:
            return False
        t0, x0, y0 = self.history[0]
        if (t - t0) < self.window_s:
            return False  # 아직 window_s만큼 지켜보지 못함 (막 시작했거나 막 리셋됨)
        _, x1, y1 = self.history[-1]
        return bool(np.hypot(x1 - x0, y1 - y0) < self.min_displacement_mm)


# [주의] 이 값은 반드시 (INFLATE_CELLS * GRID_RESOLUTION_MM = 경로가 벽에 붙어서 지나갈 수 있는
# 최소 거리)보다 작아야 한다. 처음엔 반대로 inflate를 늘려서 맞추려 했는데, 실제 방 크기(3~4m대)
# 기준으로 inflate를 늘리면 통로 자체가 A*로 못 지나갈 만큼 좁아져 버리는 걸 실측으로 확인했다
# (inflate 2칸=free 2132셀/경로 성공 -> 7칸=free 186셀/경로 실패). 그래서 inflate는 작게 유지하고
# 이 값을 inflate 여유거리(100mm)보다 작게 낮춰서 맞췄다. 이게 더 크면 정상 경로 위에서도 매
# 사이클 비상정지가 걸리고 속도 0이면 회전도 안 되어(v=0) 제자리에서 영원히 멈추는 교착상태에
# 빠진다 (탐색 150사이클, 복귀 100사이클 내내 완전히 멈춰있는 걸로 시뮬레이션에서 실제 재현됨).
EMERGENCY_STOP_DIST_MM = 80.0
RETURN_HOME_TOLERANCE_MM = 200.0

# 동적 장애물 조기 정지(예측) 설정. 카메라는 단안이라 깊이(mm) 정보가 없어 못 쓰고, 라이다의
# 연속 스캔 사이 최근접 거리 변화율(접근속도)만으로 계산한다 - Path_Planning/localize_and_plan.py
# 의 동일 로직과 통일. 정식 물체 추적/속도추정이 아니라 "최근접점 거리의 변화율"이라 로봇
# 자신의 움직임과 장애물의 움직임을 구분하지 못하지만, "간격이 위험하게 빨리 좁혀지고 있다"는
# 신호로는 충분히 유효하다.
MIN_CLOSING_SPEED_FOR_PREDICTION_MM_S = 200.0  # 이보다 느리게 좁혀지면 노이즈로 보고 무시
PREDICTIVE_STOP_TTC_S = 1.0   # 이 시간 안에 부딪힐 것으로 예상되면(거리/접근속도) 조기 정지

# 전/후/좌/우 네 방향에 확인된 장애물(OCCUPIED)이 있는지 검사할 거리. 차량 축간거리
# (WHEELBASE_MM) 정도면 실제로 그 방향으로 못 움직인다고 보기에 충분한 여유.
BOXED_IN_CHECK_DIST_MM = 150.0

# Drive(Steer/Throttle)와 주고받는 상태 파일. Path_Planning과 완전히 동일한 스키마/경로를 써서
# Drive 쪽 코드는 손댈 필요가 없음 (Camera가 안 켜져 있으면 Steer가 알아서 lidar_steering만 사용)
# (이 파일 위치: Testing/Explore_Map/explore_and_map.py -> 한 단계 위가 Testing/)
TESTING_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATE_DIR = os.path.join(TESTING_DIR, "state")
SPEED_STATE_FILE = os.path.join(STATE_DIR, "speed_command.json")
LIDAR_STEERING_FILE = os.path.join(STATE_DIR, "lidar_steering.json")


# ----------------------------
# 실시간 라이다(LD06) 읽기 - Path_Planning/localize_and_plan.py 와 동일
# ----------------------------
def parse_ld06_packet(data):
    if data[0] != 0x54 or data[1] != 0x2C:
        return None
    speed = struct.unpack('<H', data[2:4])[0] / 100.0
    start_angle = struct.unpack('<H', data[4:6])[0] / 100.0
    points = []
    for i in range(12):
        offset = 6 + i * 3
        distance = struct.unpack('<H', data[offset:offset+2])[0]
        intensity = data[offset+2]
        points.append((distance, intensity))
    end_angle = struct.unpack('<H', data[42:44])[0] / 100.0
    timestamp = struct.unpack('<H', data[44:46])[0]
    return {"speed": speed, "start_angle": start_angle, "end_angle": end_angle,
            "points": points, "timestamp": timestamp}


def ld06_point_angles(start_angle, end_angle, n_points):
    diff = end_angle - start_angle
    if diff < 0:
        diff += 360.0
    step = diff / (n_points - 1) if n_points > 1 else 0.0
    return [(start_angle + step * i) % 360.0 for i in range(n_points)]


def open_lidar():
    """실제 LD06 라이다 시리얼 포트를 연다 (라즈베리파이 전용).
    라이다가 USB 어댑터로 연결됨 - 어댑터가 회전 모터 구동을 자체 처리하므로
    GPIO PWM 제어는 더 이상 필요 없음. 시리얼 데이터만 USB로 받음."""
    import serial
    # 다른 USB 시리얼 장치가 꽂혀있으면 순서가 바뀔 수 있음 (ls /dev/ttyUSB* 로 확인)
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=230400, timeout=1)
    return ser


def close_lidar(ser):
    ser.close()


def sync_to_ld06_header(ser):
    """스트림에서 0x54,0x2C 헤더 두 바이트가 연속으로 나오는 지점까지 한 바이트씩 읽어서 맞춘다.
    시리얼 포트를 열었을 때 라이다는 이미 계속 스캔 중이라 스트림 중간부터 읽기 시작하게 되는데,
    패킷 크기(47바이트)와 read(47) 크기가 같아서 한 번 어긋나면 그 오프셋이 영원히 고정돼버려
    (자연 정렬될 기회가 없음) parse_ld06_packet이 매번 헤더 불일치로 실패하는 문제가 있었다.
    반환: 헤더를 찾으면 True, 타임아웃(라이다 연결 끊김 등)으로 못 찾으면 False"""
    prev = None
    while True:
        b = ser.read(1)
        if len(b) != 1:
            return False
        cur = b[0]
        if prev == 0x54 and cur == 0x2C:
            return True
        prev = cur


def read_one_rotation(ser, min_points=60, max_wait_s=3.0):
    """대략 한 바퀴(360도) 분량의 (angle_deg, distance_mm) 점을 모아서 반환."""
    points = []
    prev_start_angle = None
    t0 = time.time()
    while True:
        if time.time() - t0 > max_wait_s:
            break
        if not sync_to_ld06_header(ser):
            continue
        rest = ser.read(45)
        if len(rest) != 45:
            continue
        raw = bytes([0x54, 0x2C]) + rest
        result = parse_ld06_packet(raw)
        if result is None:
            continue
        angles = ld06_point_angles(result["start_angle"], result["end_angle"], len(result["points"]))
        for angle_deg, (distance, _intensity) in zip(angles, result["points"]):
            points.append((angle_deg, distance))
        if prev_start_angle is not None and result["start_angle"] < prev_start_angle and len(points) >= min_points:
            break
        prev_start_angle = result["start_angle"]
    return points


def polar_to_xy(angle_deg, dist_mm):
    angle_rad = np.deg2rad(angle_deg)
    valid = (dist_mm > MIN_RANGE_MM) & (dist_mm < MAX_RANGE_MM)
    angle_rad, dist_mm = angle_rad[valid], dist_mm[valid]
    x = dist_mm * np.cos(angle_rad)
    y = dist_mm * np.sin(angle_rad)
    return np.stack([x, y], axis=1)


def scan_points_to_xy(points):
    """[(angle_deg, distance_mm), ...] -> Nx2 xy (라이다 좌표계)"""
    if not points:
        return np.empty((0, 2))
    arr = np.array(points, dtype=float)
    return polar_to_xy(arr[:, 0], arr[:, 1])


def compute_min_obstacle_dist_mm(scan_xy):
    """라이다(로봇) 원점 기준 이번 스캔에서 가장 가까운 점까지 거리.
    위치추정(ICP)이나 프론티어 탐색 없이 raw 스캔만으로 바로 계산되므로
    '빠른 반사(즉시 정지)' 판단에 쓴다."""
    return float(np.min(np.hypot(scan_xy[:, 0], scan_xy[:, 1]))) if len(scan_xy) else float("inf")


def predict_closing_stop(dist_mm, prev_dist_mm, dt_s):
    """
    라이다만으로 가능한 간단한 동적 장애물 조기 정지 판단. 직전 스캔과 이번 스캔 사이
    최근접 거리가 얼마나 빨리 좁혀지고 있는지(접근속도)로 충돌까지 남은 시간(TTC)을
    추정해서, EMERGENCY_STOP_DIST_MM에 실제로 닿기 전이라도 미리 정지시킨다.
    반환: (should_stop: bool, closing_speed_mm_s: float)
    """
    if prev_dist_mm is None or dt_s <= 0 or not np.isfinite(prev_dist_mm) or not np.isfinite(dist_mm):
        return False, 0.0
    closing_speed_mm_s = (prev_dist_mm - dist_mm) / dt_s
    if closing_speed_mm_s < MIN_CLOSING_SPEED_FOR_PREDICTION_MM_S:
        return False, closing_speed_mm_s
    time_to_collision_s = dist_mm / closing_speed_mm_s
    return time_to_collision_s < PREDICTIVE_STOP_TTC_S, closing_speed_mm_s


def downsample_points(points, max_points):
    if len(points) <= max_points:
        return points
    idx = np.random.choice(len(points), max_points, replace=False)
    return points[idx]


def transform_points(points, x, y, theta_deg):
    th = np.deg2rad(theta_deg)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return (R @ points.T).T + np.array([x, y])


# ----------------------------
# ICP (build_map.py / Path_Planning 과 동일 로직)
# ----------------------------
def best_fit_transform(A, B):
    centroid_A, centroid_B = np.mean(A, axis=0), np.mean(B, axis=0)
    AA, BB = A - centroid_A, B - centroid_B
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    return R, t


def icp(source, target, max_iterations=ICP_MAX_ITER, tolerance=ICP_TOLERANCE,
        max_corr_dist=ICP_MAX_CORRESPONDENCE_DIST, init_guess=None, target_tree=None):
    src = source.copy()
    R_total, t_total = np.eye(2), np.zeros(2)

    if init_guess is not None:
        dx, dy, dtheta_deg = init_guess
        th = np.deg2rad(dtheta_deg)
        R0 = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        t0 = np.array([dx, dy])
        src = (R0 @ src.T).T + t0
        R_total, t_total = R0, t0

    tree = target_tree if target_tree is not None else cKDTree(target)
    prev_error, mean_error = None, None
    coarse_start = max_corr_dist * 4
    for i in range(max_iterations):
        progress = i / max(1, max_iterations - 1)
        current_max_corr = coarse_start + (max_corr_dist - coarse_start) * min(1.0, progress * 2)

        distances, indices = tree.query(src)
        mask = distances < current_max_corr
        if mask.sum() < 10:
            return src, R_total, t_total, 1e9

        R, t = best_fit_transform(src[mask], target[indices[mask]])
        src = (R @ src.T).T + t
        R_total = R @ R_total
        t_total = R @ t_total + t

        fine_mask = distances < max_corr_dist
        mean_error = np.mean(distances[fine_mask]) if fine_mask.sum() >= 10 else np.mean(distances[mask])
        if prev_error is not None and abs(prev_error - mean_error) < tolerance and progress >= 0.5:
            break
        prev_error = mean_error

    return src, R_total, t_total, mean_error


def localize(current_scan, target_map, pos_candidates=None, angle_step=30, angle_candidates=None):
    """pos_candidates가 None이면 target_map 영역 전체를 5x5 격자로 자동 생성해서 전역 탐색한다
    (Path_Planning/localize_and_plan.py의 localize()와 동일한 동작 - 전역 재탐색용)."""
    if pos_candidates is None:
        min_x, max_x = target_map[:, 0].min(), target_map[:, 0].max()
        min_y, max_y = target_map[:, 1].min(), target_map[:, 1].max()
        xs = np.linspace(min_x, max_x, 5)
        ys = np.linspace(min_y, max_y, 5)
        pos_candidates = [(x, y) for x in xs for y in ys]
    if angle_candidates is None:
        angle_candidates = list(range(0, 360, angle_step))
    target_tree = cKDTree(target_map)
    best = None
    for px, py in pos_candidates:
        for theta0 in angle_candidates:
            _, R, t, err = icp(current_scan, target_map, init_guess=(px, py, theta0), target_tree=target_tree)
            theta = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
            if best is None or err < best[0]:
                best = (err, t[0], t[1], theta)
    err, x, y, theta = best
    return x, y, theta, err


def match_scan_to_map_warmstart(current_scan, map_points_icp, prev_pose):
    """
    Path_Planning/localize_and_plan.py 의 localize_warmstart()와 동일한 발상:
    직전 위치 근방만 좁게 재탐색(빠름) -> 오차가 크면 더 넓은 범위로 재탐색.
    맵이 아직 만들어지는 중이라 Path_Planning처럼 "맵 전체 5x5 격자"가 아니라
    직전 위치를 중심으로 점점 넓혀가는 방식을 씀.
    """
    px, py, ptheta = prev_pose
    narrow_pos = [(px + dx, py + dy) for dx in NARROW_POS_OFFSETS_MM for dy in NARROW_POS_OFFSETS_MM]
    narrow_angles = [(ptheta + d) % 360 for d in (-NARROW_ANGLE_SPREAD_DEG, 0.0, NARROW_ANGLE_SPREAD_DEG)]
    x, y, theta, err = localize(current_scan, map_points_icp, narrow_pos, angle_candidates=narrow_angles)
    if err <= RELOCALIZE_ERROR_THRESHOLD_MM:
        return x, y, theta, err, "narrow"

    print("  [재탐색] 좁은 범위 탐색 실패 -> 더 넓은 범위로 재탐색")
    wide_pos = [(px + dx, py + dy) for dx in WIDE_POS_OFFSETS_MM for dy in WIDE_POS_OFFSETS_MM]
    x, y, theta, err = localize(current_scan, map_points_icp, wide_pos, angle_step=WIDE_ANGLE_STEP_DEG)
    return x, y, theta, err, "wide"


def localize_with_recovery(current_scan, map_points_icp, prev_pose, consecutive_lost):
    """
    narrow -> wide(prev_pose 근방) -> (연속 GLOBAL_RELOCALIZE_AFTER_CONSECUTIVE_LOST회 이상
    실패시) global(지금까지 쌓은 맵 전체) 3단계로 위치를 찾는다.

    IMU/GPS가 없어 라이다 스캔매칭이 유일한 위치 정보원이라, match_scan_to_map_warmstart()의
    narrow/wide는 둘 다 prev_pose 근방(최대 ±450mm)만 보기 때문에 정말로 크게 위치를 놓치면
    (급가속/슬립, 스캔이 순간적으로 심하게 가려짐 등) 영원히 못 돌아올 수 있다. 그래서 실패가
    연달아 누적되면 prev_pose 근방이라는 전제 자체를 버리고 맵 전체를 다시 뒤진다.

    반환: (x, y, theta, err, mode, new_consecutive_lost, trustworthy)
    trustworthy=False면 전역 탐색까지 해봐도 못 미더운 상태 - 호출부는 이번 결과를 쓰지 말고
    정지해야 한다.
    """
    x, y, theta, err, mode = match_scan_to_map_warmstart(current_scan, map_points_icp, prev_pose)

    if err > RELOCALIZE_ERROR_THRESHOLD_MM:
        consecutive_lost += 1
        if consecutive_lost >= GLOBAL_RELOCALIZE_AFTER_CONSECUTIVE_LOST:
            print(f"  [전역 재탐색] {consecutive_lost}회 연속으로 직전 위치 근방 탐색에 실패했습니다 - "
                  f"지금까지 쌓은 맵 전체를 다시 뒤집니다.")
            x, y, theta, err = localize(current_scan, map_points_icp, pos_candidates=None,
                                         angle_step=GLOBAL_ANGLE_STEP_DEG)
            mode = "global"
    else:
        consecutive_lost = 0

    if err <= RELOCALIZE_ERROR_THRESHOLD_MM:
        consecutive_lost = 0

    trustworthy = err <= LOCALIZATION_LOST_ERROR_THRESHOLD_MM
    return x, y, theta, err, mode, consecutive_lost, trustworthy


# ----------------------------
# 격자 변환 (Path_Planning과 동일)
# ----------------------------
def world_to_grid(x_mm, y_mm, origin, resolution):
    col = int((x_mm - origin[0]) / resolution)
    row = int((y_mm - origin[1]) / resolution)
    return row, col


def grid_to_world(row, col, origin, resolution):
    x_mm = col * resolution + origin[0]
    y_mm = row * resolution + origin[1]
    return x_mm, y_mm


def inflate_obstacles(grid, inflate_cells=2):
    return binary_dilation(grid, iterations=inflate_cells).astype(np.uint8)


# ----------------------------
# 3상태 grid: UNKNOWN=-1, FREE=0, OCCUPIED=1
# ----------------------------
def init_explore_grid():
    size = int(2 * GRID_HALF_SIZE_MM / GRID_RESOLUTION_MM) + 1
    grid = np.full((size, size), -1, dtype=np.int8)
    origin = (-GRID_HALF_SIZE_MM, -GRID_HALF_SIZE_MM)
    return grid, origin, GRID_RESOLUTION_MM


def raytrace_update(grid, origin, resolution, robot_xy, scan_world_xy):
    """로봇 위치 ~ 각 스캔 끝점 사이는 FREE, 끝점은 OCCUPIED로 표시.
    이미 OCCUPIED로 확정된 칸은 지나가는 광선 때문에 다시 FREE로 덮어쓰지 않음(벽은 안 움직이므로)."""
    h, w = grid.shape
    rx, ry = robot_xy
    for ex, ey in scan_world_xy:
        dist = np.hypot(ex - rx, ey - ry)
        steps = max(2, int(dist / (resolution / 2)))
        for i in range(steps):
            t = i / steps
            x = rx + (ex - rx) * t
            y = ry + (ey - ry) * t
            row, col = world_to_grid(x, y, origin, resolution)
            if 0 <= row < h and 0 <= col < w and grid[row, col] != 1:
                grid[row, col] = 0
        row, col = world_to_grid(ex, ey, origin, resolution)
        if 0 <= row < h and 0 <= col < w:
            grid[row, col] = 1

    close_free_gaps(grid)


def close_free_gaps(grid):
    """
    라이다는 각도 간격(1도 안팎)으로 점을 찍기 때문에, 로봇에서 멀어질수록 인접한 두 광선
    사이의 실제 간격(호의 길이)이 grid 한 칸 크기보다 커져서 두 광선 사이에 UNKNOWN 얼룩이
    낀 것처럼 남는 경우가 생긴다 (실제로는 다 빈 공간인데). 이걸 놔두면 그 얼룩이
    to_binary_obstacle_grid()에서 장애물 취급돼서 로봇 바로 옆 프론티어까지도 A*가
    막혀버릴 수 있음. FREE 마스크에 morphological closing(팽창 후 침식)을 적용해서
    1칸짜리 구멍을 메움 - 이미 확정된 OCCUPIED는 건드리지 않고 UNKNOWN만 FREE로 승격.
    """
    from scipy.ndimage import binary_closing
    free_mask = grid == 0
    closed = binary_closing(free_mask, structure=np.ones((3, 3)), iterations=1)
    grid[(grid == -1) & closed] = 0


def to_binary_obstacle_grid(grid):
    """A*용 2상태 변환: FREE(0)만 통행 가능, UNKNOWN+OCCUPIED는 장애물(1) 취급
    (안 가본 곳으로 A*가 뚫고 지나가면 안 되므로 미탐색도 장애물처럼 다룸)"""
    return np.where(grid == 0, 0, 1).astype(np.uint8)


def build_planning_grid(grid, inflate_cells=INFLATE_CELLS):
    """
    A*가 실제로 쓸 최종 grid를 만듦. 주의: UNKNOWN은 "장애물"이 아니라 그냥 "아직 모름"이라
    안전거리를 둘 실체가 없다. 그런데 프론티어는 정의상 항상 UNKNOWN 바로 옆의 FREE 칸이라서,
    inflate_obstacles()로 UNKNOWN까지 같이 부풀려버리면 모든 프론티어가 부풀려진 영역에
    파묻혀서 영원히 못 가는 목표가 돼버린다 (실제로 시뮬레이션에서 재현된 버그). 그래서
    실제로 확인된 장애물(OCCUPIED)만 부풀리고, UNKNOWN은 부풀리지 않은 채로 그대로
    "가본 적 없으니 지나가면 안 됨"만 적용한다.
    """
    inflated_occupied = binary_dilation(grid == 1, iterations=inflate_cells)
    unknown = grid == -1
    return (inflated_occupied | unknown).astype(np.uint8)


# ----------------------------
# 이동 가능 여부(물리적으로 막힘) 판정
# ----------------------------
def is_boxed_in(grid, origin, resolution, x, y, theta_deg, check_dist_mm=BOXED_IN_CHECK_DIST_MM):
    """
    로봇이 현재 위치·방향 기준 전/후/좌/우 어느 쪽으로도 못 움직이는지 확인.
    실제로 확인된 장애물(OCCUPIED=1)로 네 방향이 전부 막혀 있어야만 True.
    미탐색(UNKNOWN)은 "아직 안 가봤을 뿐"이라 막힌 것으로 치지 않음 - 그래야 그냥 미지 영역을
    UNKNOWN을 막힘으로 오판해서 탐색을 너무 일찍 포기하는 걸 방지.
    """
    h, w = grid.shape
    theta = np.deg2rad(theta_deg)
    directions = (
        (np.cos(theta), np.sin(theta)),    # 전방
        (-np.cos(theta), -np.sin(theta)),  # 후방
        (-np.sin(theta), np.cos(theta)),   # 좌측
        (np.sin(theta), -np.cos(theta)),   # 우측
    )
    for dx, dy in directions:
        nx, ny = x + dx * check_dist_mm, y + dy * check_dist_mm
        row, col = world_to_grid(nx, ny, origin, resolution)
        if not (0 <= row < h and 0 <= col < w) or grid[row, col] != 1:
            return False  # 이 방향은 확인된 장애물이 아님 -> 아직 갈 수 있는 여지가 있음
    return True  # 네 방향 전부 확인된 장애물


# ----------------------------
# 프론티어(미탐색 경계) 탐색
# ----------------------------
def find_frontier_mask(grid):
    free_mask = grid == 0
    unknown_mask = grid == -1
    unknown_dilated = binary_dilation(unknown_mask, iterations=1)
    return free_mask & unknown_dilated


def cluster_frontiers(frontier_mask, min_cluster_cells=FRONTIER_MIN_CLUSTER_CELLS):
    """
    각 프론티어 뭉치(연결된 셀들)를 대표하는 (row, col) 하나씩을 반환.
    뭉치의 평균 좌표(centroid)는 뭉치 모양이 휘어있으면(초승달 모양 등) 그 자체로는
    프론티어 셀이 아닐 수 있어서(실제로 재현된 버그), 평균과 가장 가까운 "실제 뭉치 구성원"
    셀로 스냅한다 - 그래야 반환값이 항상 진짜 FREE+UNKNOWN인접 셀임이 보장됨.
    """
    labeled, n = label(frontier_mask)
    reps = []
    for i in range(1, n + 1):
        cells = np.argwhere(labeled == i)
        if len(cells) < min_cluster_cells:
            continue
        centroid = cells.mean(axis=0)
        d = np.hypot(cells[:, 0] - centroid[0], cells[:, 1] - centroid[1])
        nearest = cells[np.argmin(d)]
        reps.append((int(nearest[0]), int(nearest[1])))
    return reps


def find_nearby_passable_cell(inflated_obstacle_grid, row, col, max_radius=4):
    """
    (row,col) 자체가 안전거리(inflate) 때문에 막혀있으면, 그 주변에서 실제로 갈 수 있는
    가장 가까운 칸을 찾아서 대신 반환. 프론티어는 정의상 벽(UNKNOWN 경계)과 맞닿아 있을 수
    있는데, 벽 바로 옆이라 로봇 안전거리(inflate) 안에 들어가 있으면 그 지점 자체엔 못 가는 게
    당연하므로, 완전히 포기하지 않고 조금 물러난 지점을 목표로 삼기 위함.
    """
    h, w = inflated_obstacle_grid.shape
    if 0 <= row < h and 0 <= col < w and inflated_obstacle_grid[row, col] == 0:
        return row, col
    for radius in range(1, max_radius + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if max(abs(dr), abs(dc)) != radius:
                    continue  # 이전 반경에서 이미 확인한 칸은 건너뜀
                nr, nc = row + dr, col + dc
                if 0 <= nr < h and 0 <= nc < w and inflated_obstacle_grid[nr, nc] == 0:
                    return nr, nc
    return None


def choose_frontier_goal(grid, robot_xy, origin, resolution, inflated_obstacle_grid,
                          avoid_points=None, avoid_radius_mm=0.0):
    """
    반환: (goal_xy 또는 None, reason)
    reason: "ok" | "no_frontier"(더 갈 미지영역 없음=탐색완료) | "unreachable"(후보는 있는데 다 막힘)

    avoid_points: [(x_mm, y_mm), ...] - 이 지점들 근방(avoid_radius_mm 이내)의 프론티어는 후보에서
    제외한다. 라이다가 못 보는 낮은 장애물에 걸려 정지(stall)했던 지점을 다시 목표로 잡지 않기
    위함 - 지도상으로는 여전히 "갈 수 있는 것처럼" 보이므로 A*만으로는 걸러지지 않는다.
    """
    frontier_mask = find_frontier_mask(grid)
    clusters = cluster_frontiers(frontier_mask)
    if not clusters:
        return None, "no_frontier"

    rx, ry = robot_xy

    def dist_key(rc):
        wx, wy = grid_to_world(rc[0], rc[1], origin, resolution)
        return np.hypot(wx - rx, wy - ry)

    clusters.sort(key=dist_key)
    start_rc = world_to_grid(rx, ry, origin, resolution)

    for row, col in clusters:
        wx, wy = grid_to_world(row, col, origin, resolution)
        if avoid_points and any(np.hypot(wx - ax, wy - ay) <= avoid_radius_mm for ax, ay in avoid_points):
            continue
        goal_cell = find_nearby_passable_cell(inflated_obstacle_grid, row, col)
        if goal_cell is None:
            continue
        if astar(inflated_obstacle_grid, start_rc, goal_cell) is not None:
            return grid_to_world(goal_cell[0], goal_cell[1], origin, resolution), "ok"

    return None, "unreachable"


# ----------------------------
# 경로 -> 조향 (Path_Planning과 동일)
# ----------------------------
def pure_pursuit_steering(path_world, x, y, theta_deg, lookahead_mm=LOOKAHEAD_MM,
                           wheelbase_mm=WHEELBASE_MM, max_steering_deg=MAX_STEERING_DEG):
    path = np.asarray(path_world, dtype=float)
    if len(path) == 0:
        return 0.0
    dists = np.hypot(path[:, 0] - x, path[:, 1] - y)
    ahead_idx = np.where(dists >= lookahead_mm)[0]
    target = path[ahead_idx[0]] if len(ahead_idx) else path[-1]

    target_heading_deg = np.degrees(np.arctan2(target[1] - y, target[0] - x))
    alpha_deg = (target_heading_deg - theta_deg + 180) % 360 - 180
    alpha = np.deg2rad(alpha_deg)

    actual_lookahead = max(1.0, np.hypot(target[0] - x, target[1] - y))
    curvature = 2.0 * np.sin(alpha) / actual_lookahead
    steering_deg = np.degrees(np.arctan(wheelbase_mm * curvature))
    return max(-max_steering_deg, min(max_steering_deg, steering_deg))


# 급회전 구간에서 순항 속도를 낮추는 배율. 1.0=조향각 0(직진), CURVE_SPEED_MIN_FACTOR=조향각
# 최대(가장 급한 회전)일 때. Path_Planning/localize_and_plan.py의 동일 로직과 통일.
CURVE_SPEED_MIN_FACTOR = 0.5


def curvature_speed_factor(steering_deg, max_steering_deg=EXPLORE_MAX_STEERING_DEG):
    """조향각 크기(=회전이 얼마나 급한지)에 비례해서 속도 배율(CURVE_SPEED_MIN_FACTOR~1.0)을 낮춘다."""
    ratio = min(1.0, abs(steering_deg) / max_steering_deg)
    return 1.0 - ratio * (1.0 - CURVE_SPEED_MIN_FACTOR)


def astar(grid, start, goal):
    """grid: 0=빈공간, 1=장애물. start/goal: (row, col)"""
    h, w = grid.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w and 0 <= goal[0] < h and 0 <= goal[1] < w):
        return None
    if grid[start] == 1 or grid[goal] == 1:
        return None

    def heuristic(a, b):
        return np.hypot(a[0]-b[0], a[1]-b[1])

    neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for dr, dc in neighbors:
            nr, nc = current[0]+dr, current[1]+dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if grid[nr, nc] == 1:
                continue
            step_cost = np.hypot(dr, dc)
            tentative_g = g_score[current] + step_cost
            if (nr, nc) not in g_score or tentative_g < g_score[(nr, nc)]:
                g_score[(nr, nc)] = tentative_g
                f = tentative_g + heuristic((nr, nc), goal)
                heapq.heappush(open_set, (f, (nr, nc)))
                came_from[(nr, nc)] = current
    return None


# ----------------------------
# Drive(Steer/Throttle)로 명령 발행 - Path_Planning과 완전히 동일한 파일/스키마
# ----------------------------
def publish_speed_command(speed_percent, min_obstacle_dist_mm, goal_reached=False):
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp_path = SPEED_STATE_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump({
            "speed_percent": speed_percent,
            "min_obstacle_dist_mm": min_obstacle_dist_mm,
            "goal_reached": goal_reached,
            "timestamp": time.time(),
        }, f)
    os.replace(tmp_path, SPEED_STATE_FILE)


def publish_lidar_steering(angle_deg):
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp_path = LIDAR_STEERING_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump({"angle_deg": angle_deg, "timestamp": time.time()}, f)
    os.replace(tmp_path, LIDAR_STEERING_FILE)


# ----------------------------
# 맵 내보내기 - build_map.py와 동일한 산출물 포맷 (Path_Planning이 그대로 읽을 수 있게)
# ----------------------------
def export_map(out_dir, map_points, grid, origin, resolution):
    os.makedirs(out_dir, exist_ok=True)
    export_grid = to_binary_obstacle_grid(grid)  # 미탐색은 안전하게 장애물 취급해서 내보냄

    pd.DataFrame(map_points, columns=["x_mm", "y_mm"]).to_csv(os.path.join(out_dir, "map_points.csv"), index=False)
    np.save(os.path.join(out_dir, "map_occupancy_grid.npy"), export_grid)
    pd.DataFrame([{
        "origin_x_mm": origin[0], "origin_y_mm": origin[1],
        "resolution_mm": resolution, "width": export_grid.shape[1], "height": export_grid.shape[0],
    }]).to_csv(os.path.join(out_dir, "map_grid_meta.csv"), index=False)

    png_saved = False
    try:
        # matplotlib.use("TkAgg")는 지연 로딩이라(파일 맨 위 참고) 여기서 실제로 그려보기 전까진
        # 실패를 알 수 없다 - Path_Planning이 실제로 필요로 하는 건 위 3개 데이터 파일이지 이
        # png가 아니므로, 그림이 실패해도 이미 저장된 데이터 파일까지 잃으면 안 된다.
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(export_grid, cmap="gray_r", origin="upper")
        ax.set_title("Explore_Map result (unknown -> obstacle)")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "map_result.png"), dpi=150)
        plt.close(fig)
        png_saved = True
    except Exception as e:
        print(f"  [안내] map_result.png 저장에 실패했습니다({e}) - 맵 데이터 파일은 정상 저장됐습니다.")

    saved = "map_points.csv, map_occupancy_grid.npy, map_grid_meta.csv" + (", map_result.png" if png_saved else "")
    print(f"  맵 저장 완료: {out_dir} ({saved})")


# ----------------------------
# 실시간 시각화 - 점유 격자 + 이동 경로 + 현재 위치/방향 + 목표(프론티어/원점)
# ----------------------------
class LiveMapView:
    """탐색/복귀 중 matplotlib 창 하나로 상태를 매 사이클 갱신해서 보여준다.
    인터랙티브 백엔드(TkAgg 등)가 없는 헤드리스 환경에서는 이 클래스를 아예 만들지 않고
    건너뛰도록 main()에서 처리한다."""

    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.path_history = []

    def update(self, grid, origin, resolution, pose, goal_xy=None, title=""):
        x, y, theta = pose
        self.path_history.append((x, y))

        self.ax.clear()
        # UNKNOWN(-1)=회색, FREE(0)=흰색, OCCUPIED(1)=검정으로 표시
        display_grid = np.where(grid == -1, 0.5, np.where(grid == 1, 1.0, 0.0))
        h, w = grid.shape
        extent = [origin[0], origin[0] + w * resolution, origin[1] + h * resolution, origin[1]]
        self.ax.imshow(display_grid, cmap="gray_r", vmin=0, vmax=1, extent=extent, origin="upper")

        if len(self.path_history) > 1:
            px, py = zip(*self.path_history)
            self.ax.plot(px, py, "-", color="dodgerblue", linewidth=1.5, label="이동 경로")

        self.ax.plot(x, y, "o", color="red", markersize=8, label="현재 위치")
        heading_len = 200.0
        hx = x + heading_len * np.cos(np.deg2rad(theta))
        hy = y + heading_len * np.sin(np.deg2rad(theta))
        self.ax.annotate("", xy=(hx, hy), xytext=(x, y),
                          arrowprops=dict(arrowstyle="->", color="red"))

        if goal_xy is not None:
            self.ax.plot(goal_xy[0], goal_xy[1], "*", color="lime", markersize=14, label="목표")

        self.ax.plot(0, 0, "s", color="orange", markersize=8, label="원점")
        self.ax.set_title(title)
        self.ax.legend(loc="upper right", fontsize=8)
        self.ax.set_aspect("equal")

        try:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
        except Exception:
            pass  # 창이 닫히는 등 렌더링 실패는 탐색 자체를 막을 이유가 없으므로 무시

    def close(self):
        plt.ioff()
        plt.close(self.fig)


# ----------------------------
# 명령 기록 - 매 사이클 실제로 내려간 조향/속도 명령을 CSV로 남긴다.
# 차가 직진 명령인데도 한쪽으로 쏠린다면, 이 로그에서 steering_deg가 계속 한쪽 부호로만
# 찍히는지(=소프트웨어가 애초에 삐딱한 조향을 명령함) 아니면 steering_deg는 0 근방인데도
# 실제로는 쏠린다는 별도 관찰(=조향각 자체는 정상인데 구동축/얼라인먼트 등 기구적 문제)인지를
# 나중에 CSV만 열어봐도 바로 구분할 수 있게 하기 위함.
# ----------------------------
class CommandLog:
    def __init__(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir, "command_log.csv")
        self._f = open(self.path, "w", newline="")
        self._writer = csv.writer(self._f)
        self._writer.writerow(["timestamp", "phase", "cycle", "x_mm", "y_mm", "theta_deg",
                                "localize_mode", "localize_err_mm", "steering_deg",
                                "speed_percent", "min_obstacle_dist_mm", "note"])
        self._f.flush()

    def log(self, phase, cycle, x, y, theta, mode, err, steering_deg, speed_percent,
            min_obstacle_dist_mm, note=""):
        self._writer.writerow([time.time(), phase, cycle, x, y, theta, mode, err,
                                steering_deg, speed_percent, min_obstacle_dist_mm, note])
        self._f.flush()  # Ctrl+C 등으로 중간에 죽어도 그때까지 기록은 남도록 매번 flush

    def close(self):
        self._f.close()


class _NullCommandLog:
    """CommandLog를 안 넘겼을 때(테스트 등) 조용히 아무것도 안 하는 대역."""

    def log(self, *args, **kwargs):
        pass

    def close(self):
        pass


NULL_COMMAND_LOG = _NullCommandLog()


# ----------------------------
# 탐색 루프
# ----------------------------
def run_exploration(read_scan_fn, max_cycles=MAX_EXPLORE_CYCLES, live_view=None, cmd_log=NULL_COMMAND_LOG,
                     stall=None):
    """
    read_scan_fn(): 다음 스캔의 [(angle_deg, distance_mm), ...] 를 반환하는 함수.
    실제 하드웨어에서는 read_one_rotation(ser)를 감싼 함수, 테스트에서는 가상 스캔 생성 함수를 넣음.
    stall: StallDetector 주입용 (생략하면 기본 설정으로 새로 만듦) - 테스트에서 window_s를 짧게
    줄여서 검증할 때 씀. StallDetector.__init__의 기본 인자는 모듈 임포트 시점에 한 번만 바인딩돼서
    이후 STALL_CHECK_WINDOW_S를 monkeypatch해도 반영이 안 되므로, 주입 방식으로 우회한다.
    반환: (map_points, grid, origin, resolution, final_pose, interrupted)
    interrupted=True면 Ctrl+C로 도중에 중단된 것 - 그때까지 쌓은 데이터는 정상적으로 반환하므로
    호출부(main())는 이 경우에도 export_map()으로 저장할 수 있다.
    """
    grid, origin, resolution = init_explore_grid()
    map_points = np.empty((0, 2))
    pose = (0.0, 0.0, 0.0)
    min_obstacle_dist_mm = float("inf")
    consecutive_lost = 0
    interrupted = False
    stall = stall if stall is not None else StallDetector()
    avoid_points = []  # 후진으로 탈출했던 지점들 - 프론티어 후보에서 계속 제외

    try:
        for cycle in range(max_cycles):
            scan_points = read_scan_fn()
            current_scan = scan_points_to_xy(scan_points)
            if len(current_scan) < 10:
                print("  스캔 포인트가 너무 적습니다. 다음 회전을 기다립니다.")
                continue

            # --- 빠른 반사: 위치추정(ICP) 전에 raw 스캔만으로 즉시 장애물 체크 ---
            # 위치추정+grid갱신+프론티어탐색까지 포함한 전체 사이클을 기다리면 그 사이 갑자기 나타난
            # 장애물에 너무 늦게 반응하게 됨. 여기서 먼저 정지시켜두면 이번 스캔 주기(수백ms) 안에 반응함.
            fast_min_obstacle_dist_mm = compute_min_obstacle_dist_mm(current_scan)
            if fast_min_obstacle_dist_mm < EMERGENCY_STOP_DIST_MM:
                publish_speed_command(0.0, fast_min_obstacle_dist_mm, goal_reached=False)
                print(f"  [빠른 반사 정지] 장애물이 {fast_min_obstacle_dist_mm:.0f}mm 이내 감지 - "
                      f"위치추정/프론티어탐색 전에 즉시 정지 명령을 보냈습니다.")
                cmd_log.log("explore", cycle, *pose, "n/a", None, None, 0.0,
                             fast_min_obstacle_dist_mm, note="fast_stop")

            if cycle == 0:
                x, y, theta, err, mode = 0.0, 0.0, 0.0, 0.0, "origin"
            else:
                map_icp_target = downsample_points(map_points, MAP_ICP_DOWNSAMPLE)
                x, y, theta, err, mode, consecutive_lost, trustworthy = localize_with_recovery(
                    current_scan, map_icp_target, pose, consecutive_lost)
                if not trustworthy:
                    print(f"  [위치추정 실패] 오차 {err:.0f}mm - 전역 탐색까지 해봐도 위치를 신뢰할 수 "
                          f"없습니다. 이번 사이클은 정지하고, 맵/위치는 갱신하지 않습니다.")
                    publish_speed_command(0.0, compute_min_obstacle_dist_mm(current_scan), goal_reached=False)
                    cmd_log.log("explore", cycle, *pose, mode, err, None, 0.0,
                                 compute_min_obstacle_dist_mm(current_scan), note="localization_lost")
                    continue  # pose/map/grid 그대로 - 다음 사이클도 직전 신뢰 위치 근방부터 다시 시도

            pose = (x, y, theta)
            print(f"[cycle {cycle}] 위치({mode}): x={x:.0f}mm y={y:.0f}mm theta={theta:.1f}deg err={err:.1f}mm")
            stall.update(time.time(), x, y)

            scan_world = transform_points(current_scan, x, y, theta)
            map_points = np.vstack([map_points, scan_world]) if len(map_points) else scan_world
            raytrace_update(grid, origin, resolution, (x, y), scan_world)

            min_obstacle_dist_mm = compute_min_obstacle_dist_mm(current_scan)
            base_speed_percent = 0.0 if min_obstacle_dist_mm < EMERGENCY_STOP_DIST_MM else CRUISE_SPEED_PERCENT_EXPLORE

            if is_boxed_in(grid, origin, resolution, x, y, theta):
                publish_speed_command(0.0, min_obstacle_dist_mm, goal_reached=False)
                cmd_log.log("explore", cycle, x, y, theta, mode, err, None, 0.0,
                             min_obstacle_dist_mm, note="boxed_in")
                if live_view is not None:
                    live_view.update(grid, origin, resolution, pose, goal_xy=None,
                                      title=f"탐색 cycle {cycle} - 전/후/좌/우 모두 막힘, 종료")
                print("  전/후/좌/우 모두 확인된 장애물로 막혀 더 이상 움직일 수 없습니다 - 탐색을 종료합니다.")
                break

            # 정체(stall) 감지: 라이다 스캔 평면보다 낮아서 안 보이는 바닥 장애물(문턱, 케이블 등)에
            # 걸리면 min_obstacle_dist_mm은 계속 멀쩡하게 나와서 위 장애물 감지로는 못 잡는다 -
            # "명령은 계속 나가는데 실제 위치가 안 바뀐다"는 것 자체를 감시해서 걸러낸다.
            if base_speed_percent > 0.0 and stall.is_stalled(time.time()):
                print(f"  [정체 감지] 최근 {STALL_CHECK_WINDOW_S:.0f}초 동안 {STALL_DISPLACEMENT_MM:.0f}mm도 "
                      f"못 움직였습니다 - 라이다가 못 보는 장애물에 걸린 것으로 보고 후진합니다.")
                publish_lidar_steering(0.0)
                publish_speed_command(STALL_REVERSE_SPEED_PERCENT, min_obstacle_dist_mm, goal_reached=False)
                cmd_log.log("explore", cycle, x, y, theta, mode, err, 0.0, STALL_REVERSE_SPEED_PERCENT,
                             min_obstacle_dist_mm, note="stall_reverse")
                if live_view is not None:
                    live_view.update(grid, origin, resolution, pose, goal_xy=None,
                                      title=f"탐색 cycle {cycle} - 정체 감지, 후진 중")
                time.sleep(STALL_REVERSE_DURATION_S)
                publish_speed_command(0.0, min_obstacle_dist_mm, goal_reached=False)
                avoid_points.append((x, y))
                stall.reset()
                continue

            inflated = build_planning_grid(grid, INFLATE_CELLS)
            goal_xy, reason = choose_frontier_goal(grid, (x, y), origin, resolution, inflated,
                                                    avoid_points=avoid_points, avoid_radius_mm=STALL_AVOID_RADIUS_MM)

            if live_view is not None:
                live_view.update(grid, origin, resolution, pose, goal_xy=goal_xy, title=f"탐색 중 - cycle {cycle}")

            if goal_xy is None:
                publish_speed_command(0.0, min_obstacle_dist_mm, goal_reached=False)
                cmd_log.log("explore", cycle, x, y, theta, mode, err, None, 0.0,
                             min_obstacle_dist_mm, note=f"stop_{reason}")
                if reason == "no_frontier":
                    print("탐색 완료 - 더 갈 미지 영역이 없습니다.")
                else:
                    print("남은 미지 영역은 있지만 전부 경로가 막혀있어 탐색을 종료합니다.")
                break

            start_rc = world_to_grid(x, y, origin, resolution)
            goal_rc = world_to_grid(goal_xy[0], goal_xy[1], origin, resolution)
            path_rc = astar(inflated, start_rc, goal_rc)
            if path_rc is None:
                floored = apply_min_moving_floor(base_speed_percent)
                publish_speed_command(floored, min_obstacle_dist_mm, goal_reached=False)
                cmd_log.log("explore", cycle, x, y, theta, mode, err, None, floored,
                             min_obstacle_dist_mm, note="no_path")
                print("  이 프론티어로 가는 경로를 못 찾았습니다. 다음 사이클에 재시도합니다.")
                continue

            path_world = [grid_to_world(r, c, origin, resolution) for r, c in path_rc]
            steering_deg = pure_pursuit_steering(path_world, x, y, theta, max_steering_deg=EXPLORE_MAX_STEERING_DEG)
            publish_lidar_steering(steering_deg)

            # 회전이 급할수록(조향각이 클수록) 순항 속도를 낮춤 - 비상정지(base_speed_percent=0)는 항상 우선됨.
            # 다만 곡률감속이나 낮은 순항속도 자체가 데드존(MIN_MOVING_SPEED_PERCENT) 밑으로 내려가면 안 되므로
            # apply_min_moving_floor로 바닥을 깐다 (정지 의도인 0.0은 그대로 통과시킴).
            speed_percent = apply_min_moving_floor(base_speed_percent * curvature_speed_factor(steering_deg))
            publish_speed_command(speed_percent, min_obstacle_dist_mm, goal_reached=False)
            cmd_log.log("explore", cycle, x, y, theta, mode, err, steering_deg, speed_percent,
                         min_obstacle_dist_mm)
        else:
            print(f"[안내] 최대 사이클({max_cycles}) 도달 - 탐색을 중단합니다.")
    except KeyboardInterrupt:
        interrupted = True
        print("  [중단] Ctrl+C로 탐색을 중단했습니다 - 지금까지 만든 맵을 저장합니다.")

    publish_speed_command(0.0, min_obstacle_dist_mm, goal_reached=False)
    return map_points, grid, origin, resolution, pose, interrupted


def return_to_origin(read_scan_fn, map_points, grid, origin, resolution, pose, max_cycles=MAX_EXPLORE_CYCLES,
                      live_view=None, cmd_log=NULL_COMMAND_LOG):
    """완성된 지도를 이용해 A*+Pure Pursuit로 원점(0,0)까지 복귀"""
    inflated = build_planning_grid(grid, INFLATE_CELLS)
    map_icp_target = downsample_points(map_points, MAP_ICP_DOWNSAMPLE)
    consecutive_lost = 0

    try:
        for cycle in range(max_cycles):
            scan_points = read_scan_fn()
            current_scan = scan_points_to_xy(scan_points)
            if len(current_scan) < 10:
                continue

            # --- 빠른 반사: 위치추정(ICP) 전에 raw 스캔만으로 즉시 장애물 체크 (복귀 중에도 동일하게 적용) ---
            fast_min_obstacle_dist_mm = compute_min_obstacle_dist_mm(current_scan)
            if fast_min_obstacle_dist_mm < EMERGENCY_STOP_DIST_MM:
                publish_speed_command(0.0, fast_min_obstacle_dist_mm, goal_reached=False)
                print(f"  [빠른 반사 정지] 장애물이 {fast_min_obstacle_dist_mm:.0f}mm 이내 감지 - "
                      f"위치추정 전에 즉시 정지 명령을 보냈습니다.")
                cmd_log.log("return", cycle, *pose, "n/a", None, None, 0.0,
                             fast_min_obstacle_dist_mm, note="fast_stop")

            x, y, theta, err, mode, consecutive_lost, trustworthy = localize_with_recovery(
                current_scan, map_icp_target, pose, consecutive_lost)
            if not trustworthy:
                print(f"  [위치추정 실패] 오차 {err:.0f}mm - 전역 탐색까지 해봐도 위치를 신뢰할 수 "
                      f"없습니다. 이번 사이클은 정지하고, 위치는 갱신하지 않습니다.")
                publish_speed_command(0.0, compute_min_obstacle_dist_mm(current_scan), goal_reached=False)
                cmd_log.log("return", cycle, *pose, mode, err, None, 0.0,
                             compute_min_obstacle_dist_mm(current_scan), note="localization_lost")
                continue  # pose 그대로 - 다음 사이클도 직전 신뢰 위치 근방부터 다시 시도

            pose = (x, y, theta)
            print(f"[복귀] 위치({mode}): x={x:.0f}mm y={y:.0f}mm theta={theta:.1f}deg")

            if live_view is not None:
                live_view.update(grid, origin, resolution, pose, goal_xy=(0.0, 0.0), title="원점으로 복귀 중")

            dist_to_home = np.hypot(x, y)
            if dist_to_home <= RETURN_HOME_TOLERANCE_MM:
                publish_speed_command(0.0, float("inf"), goal_reached=True)
                cmd_log.log("return", cycle, x, y, theta, mode, err, None, 0.0, float("inf"), note="arrived")
                print(f"원점 복귀 완료 (남은 거리 {dist_to_home:.0f}mm).")
                return True

            min_obstacle_dist_mm = compute_min_obstacle_dist_mm(current_scan)
            base_speed_percent = 0.0 if min_obstacle_dist_mm < EMERGENCY_STOP_DIST_MM else CRUISE_SPEED_PERCENT_EXPLORE

            if is_boxed_in(grid, origin, resolution, x, y, theta):
                publish_speed_command(0.0, min_obstacle_dist_mm, goal_reached=False)
                cmd_log.log("return", cycle, x, y, theta, mode, err, None, 0.0,
                             min_obstacle_dist_mm, note="boxed_in")
                print("  전/후/좌/우 모두 확인된 장애물로 막혀 더 이상 움직일 수 없습니다 - 복귀를 중단합니다.")
                return False

            start_rc = world_to_grid(x, y, origin, resolution)
            goal_rc = world_to_grid(0.0, 0.0, origin, resolution)
            path_rc = astar(inflated, start_rc, goal_rc)
            if path_rc is None:
                floored = apply_min_moving_floor(base_speed_percent)
                publish_speed_command(floored, min_obstacle_dist_mm, goal_reached=False)
                cmd_log.log("return", cycle, x, y, theta, mode, err, None, floored,
                             min_obstacle_dist_mm, note="no_path")
                print("  복귀 경로를 못 찾았습니다. 다음 스캔에서 재시도합니다.")
                continue

            path_world = [grid_to_world(r, c, origin, resolution) for r, c in path_rc]
            steering_deg = pure_pursuit_steering(path_world, x, y, theta, max_steering_deg=EXPLORE_MAX_STEERING_DEG)
            publish_lidar_steering(steering_deg)

            speed_percent = apply_min_moving_floor(base_speed_percent * curvature_speed_factor(steering_deg))
            publish_speed_command(speed_percent, min_obstacle_dist_mm, goal_reached=False)
            cmd_log.log("return", cycle, x, y, theta, mode, err, steering_deg, speed_percent, min_obstacle_dist_mm)
    except KeyboardInterrupt:
        print("  [중단] Ctrl+C로 복귀를 중단했습니다.")
        publish_speed_command(0.0, float("inf"), goal_reached=False)
        return False

    print("[안내] 복귀 최대 사이클 도달 - 복귀를 완료하지 못했습니다.")
    return False


def main():
    parser = argparse.ArgumentParser(description="초기 위치만 지정하면 혼자 탐색하며 지도를 만들고 원점으로 복귀")
    parser.add_argument("--out", default="./map_output", help="완성된 맵을 저장할 폴더 (기본 ./map_output)")
    parser.add_argument("--max_cycles", type=int, default=MAX_EXPLORE_CYCLES,
                         help=f"탐색/복귀 각각의 최대 사이클 수 (기본 {MAX_EXPLORE_CYCLES}, 무한루프 방지용)")
    parser.add_argument("--no_live_view", action="store_true",
                         help="실시간 지도/경로 시각화 창을 띄우지 않음 (디스플레이 없는 헤드리스 환경용)")
    args = parser.parse_args()

    print("=== 실시간 라이다로 자율 탐색+매핑 시작 (Ctrl+C 종료) ===")
    ser = open_lidar()

    live_view = None
    if not args.no_live_view and matplotlib.get_backend().lower() != "agg":
        # matplotlib.use("TkAgg")는 지연 로딩이라 여기서 실제로 창을 만들어보기 전까지는
        # 디스플레이가 없어도 실패하지 않는다 - 그래서 진짜 실패 여부는 LiveMapView() 생성
        # 시점에 try/except로 잡아야 한다 (헤드리스 SSH 환경에서 곧바로 죽는 문제 있었음).
        try:
            live_view = LiveMapView()
        except Exception as e:
            print(f"[안내] 실시간 시각화 창을 열지 못해({e}) 시각화 없이 진행합니다.")
            live_view = None
    elif not args.no_live_view:
        print("[안내] 인터랙티브 디스플레이를 찾지 못해 실시간 시각화 없이 진행합니다.")

    def read_scan():
        return read_one_rotation(ser)

    cmd_log = CommandLog(args.out)
    try:
        map_points, grid, origin, resolution, pose, interrupted = run_exploration(
            read_scan, args.max_cycles, live_view=live_view, cmd_log=cmd_log)
        export_map(args.out, map_points, grid, origin, resolution)
        if interrupted:
            print(f"탐색이 도중에 중단되어 여기까지 만든 맵만 저장했습니다 ({args.out}) - 원점 복귀는 생략합니다.")
        else:
            print("\n=== 원점으로 복귀 시작 ===")
            return_to_origin(read_scan, map_points, grid, origin, resolution, pose, args.max_cycles,
                              live_view=live_view, cmd_log=cmd_log)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        close_lidar(ser)
        if live_view is not None:
            live_view.close()
        cmd_log.close()
        print(f"명령 기록: {cmd_log.path}")


if __name__ == "__main__":
    main()
