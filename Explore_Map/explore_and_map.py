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
5. 더 이상 갈 프론티어가 없으면(혹은 남은 프론티어가 전부 경로가 막혀있으면) 탐색 종료 ->
   지금까지 쌓은 지도를 build_map.py와 같은 형식(map_points.csv, map_occupancy_grid.npy,
   map_grid_meta.csv)으로 저장 -> 그 지도로 원점(0,0)까지 A*+Pure Pursuit로 복귀

이 단계는 실내(차선 없음)라 Camera는 켤 필요가 없습니다 - Drive/Steer는 camera_steering.json이
없으면(또는 오래되면) 자동으로 lidar_steering.json만 보고 동작하도록 이미 만들어져 있어서
그대로 재사용됩니다. Drive(Steer/Throttle)는 이 스크립트를 실행하는 동안에도 평소처럼
Testing/state/ 의 speed_command.json, lidar_steering.json 을 읽는 방식 그대로입니다.

탐색 중에는 스캔 정합 품질을 위해 Path_Planning보다 낮은 속도(CRUISE_SPEED_PERCENT_EXPLORE)로
순항합니다.

[한계]
- 벽만 있는 빈 공간처럼 특징이 적으면 ICP가 조금씩 틀어질 수 있고(누적 보정/loop closure 없음),
  다녀본 적 없는 큰 공간에서는 GRID_HALF_SIZE_MM 로 잡아둔 고정 크기 grid 밖으로 못 나갑니다.
  지금 방 크기(수 미터) 기준으로는 여유 있게 잡아뒀습니다.

[사용법]
    python3 explore_and_map.py --out ./map_output
"""

import os
import json
import time
import argparse
import heapq
import struct
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
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

INFLATE_CELLS = 2
FRONTIER_MIN_CLUSTER_CELLS = 3   # 이보다 작은 프론티어 뭉치는 노이즈로 보고 무시
MAX_EXPLORE_CYCLES = 300          # 무한루프 방지용 안전장치

CRUISE_SPEED_PERCENT_EXPLORE = 25.0  # Path_Planning(40%)보다 느리게 - 스캔 정합 품질을 위해

# [주의] 이 값은 반드시 (INFLATE_CELLS * GRID_RESOLUTION_MM = 경로가 벽에 붙어서 지나갈 수 있는
# 최소 거리)보다 작아야 한다. 처음엔 반대로 inflate를 늘려서 맞추려 했는데, 실제 방 크기(3~4m대)
# 기준으로 inflate를 늘리면 통로 자체가 A*로 못 지나갈 만큼 좁아져 버리는 걸 실측으로 확인했다
# (inflate 2칸=free 2132셀/경로 성공 -> 7칸=free 186셀/경로 실패). 그래서 inflate는 작게 유지하고
# 이 값을 inflate 여유거리(100mm)보다 작게 낮춰서 맞췄다. 이게 더 크면 정상 경로 위에서도 매
# 사이클 비상정지가 걸리고 속도 0이면 회전도 안 되어(v=0) 제자리에서 영원히 멈추는 교착상태에
# 빠진다 (탐색 150사이클, 복귀 100사이클 내내 완전히 멈춰있는 걸로 시뮬레이션에서 실제 재현됨).
EMERGENCY_STOP_DIST_MM = 80.0
RETURN_HOME_TOLERANCE_MM = 200.0

WHEELBASE_MM = 150.0     # TODO: 실제 차량 축간거리로 교체
LOOKAHEAD_MM = 400.0
MAX_STEERING_DEG = 30.0

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
    """실제 LD06 라이다 시리얼 포트를 열고 회전 모터를 켠다 (라즈베리파이 전용)"""
    import serial
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(12, GPIO.OUT)
    pwm = GPIO.PWM(12, 50000)
    pwm.start(50)
    ser = serial.Serial(port='/dev/ttyS0', baudrate=230400, timeout=1)
    return ser, pwm, GPIO


def close_lidar(ser, pwm, GPIO):
    pwm.stop()
    GPIO.cleanup()
    ser.close()


def read_one_rotation(ser, min_points=60, max_wait_s=3.0):
    """대략 한 바퀴(360도) 분량의 (angle_deg, distance_mm) 점을 모아서 반환."""
    points = []
    prev_start_angle = None
    t0 = time.time()
    while True:
        if time.time() - t0 > max_wait_s:
            break
        raw = ser.read(47)
        if len(raw) != 47:
            continue
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


def localize(current_scan, target_map, pos_candidates, angle_step=30, angle_candidates=None):
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


def choose_frontier_goal(grid, robot_xy, origin, resolution, inflated_obstacle_grid):
    """
    반환: (goal_xy 또는 None, reason)
    reason: "ok" | "no_frontier"(더 갈 미지영역 없음=탐색완료) | "unreachable"(후보는 있는데 다 막힘)
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

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(export_grid, cmap="gray_r", origin="upper")
    ax.set_title("Explore_Map result (unknown -> obstacle)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "map_result.png"), dpi=150)
    plt.close(fig)
    print(f"  맵 저장 완료: {out_dir} (map_points.csv, map_occupancy_grid.npy, map_grid_meta.csv, map_result.png)")


# ----------------------------
# 탐색 루프
# ----------------------------
def run_exploration(read_scan_fn, max_cycles=MAX_EXPLORE_CYCLES):
    """
    read_scan_fn(): 다음 스캔의 [(angle_deg, distance_mm), ...] 를 반환하는 함수.
    실제 하드웨어에서는 read_one_rotation(ser)를 감싼 함수, 테스트에서는 가상 스캔 생성 함수를 넣음.
    반환: (map_points, grid, origin, resolution, final_pose)
    """
    grid, origin, resolution = init_explore_grid()
    map_points = np.empty((0, 2))
    pose = (0.0, 0.0, 0.0)
    min_obstacle_dist_mm = float("inf")

    for cycle in range(max_cycles):
        scan_points = read_scan_fn()
        current_scan = scan_points_to_xy(scan_points)
        if len(current_scan) < 10:
            print("  스캔 포인트가 너무 적습니다. 다음 회전을 기다립니다.")
            continue

        if cycle == 0:
            x, y, theta, err, mode = 0.0, 0.0, 0.0, 0.0, "origin"
        else:
            map_icp_target = downsample_points(map_points, MAP_ICP_DOWNSAMPLE)
            x, y, theta, err, mode = match_scan_to_map_warmstart(current_scan, map_icp_target, pose)
        pose = (x, y, theta)
        print(f"[cycle {cycle}] 위치({mode}): x={x:.0f}mm y={y:.0f}mm theta={theta:.1f}deg err={err:.1f}mm")

        scan_world = transform_points(current_scan, x, y, theta)
        map_points = np.vstack([map_points, scan_world]) if len(map_points) else scan_world
        raytrace_update(grid, origin, resolution, (x, y), scan_world)

        min_obstacle_dist_mm = (float(np.min(np.hypot(current_scan[:, 0], current_scan[:, 1])))
                                 if len(current_scan) else float("inf"))
        speed_percent = 0.0 if min_obstacle_dist_mm < EMERGENCY_STOP_DIST_MM else CRUISE_SPEED_PERCENT_EXPLORE
        publish_speed_command(speed_percent, min_obstacle_dist_mm, goal_reached=False)

        inflated = build_planning_grid(grid, INFLATE_CELLS)
        goal_xy, reason = choose_frontier_goal(grid, (x, y), origin, resolution, inflated)

        if goal_xy is None:
            if reason == "no_frontier":
                print("탐색 완료 - 더 갈 미지 영역이 없습니다.")
            else:
                print("남은 미지 영역은 있지만 전부 경로가 막혀있어 탐색을 종료합니다.")
            break

        start_rc = world_to_grid(x, y, origin, resolution)
        goal_rc = world_to_grid(goal_xy[0], goal_xy[1], origin, resolution)
        path_rc = astar(inflated, start_rc, goal_rc)
        if path_rc is None:
            print("  이 프론티어로 가는 경로를 못 찾았습니다. 다음 사이클에 재시도합니다.")
            continue

        path_world = [grid_to_world(r, c, origin, resolution) for r, c in path_rc]
        steering_deg = pure_pursuit_steering(path_world, x, y, theta)
        publish_lidar_steering(steering_deg)
    else:
        print(f"[안내] 최대 사이클({max_cycles}) 도달 - 탐색을 중단합니다.")

    publish_speed_command(0.0, min_obstacle_dist_mm, goal_reached=False)
    return map_points, grid, origin, resolution, pose


def return_to_origin(read_scan_fn, map_points, grid, origin, resolution, pose, max_cycles=MAX_EXPLORE_CYCLES):
    """완성된 지도를 이용해 A*+Pure Pursuit로 원점(0,0)까지 복귀"""
    inflated = build_planning_grid(grid, INFLATE_CELLS)
    map_icp_target = downsample_points(map_points, MAP_ICP_DOWNSAMPLE)

    for _ in range(max_cycles):
        scan_points = read_scan_fn()
        current_scan = scan_points_to_xy(scan_points)
        if len(current_scan) < 10:
            continue

        x, y, theta, err, mode = match_scan_to_map_warmstart(current_scan, map_icp_target, pose)
        pose = (x, y, theta)
        print(f"[복귀] 위치({mode}): x={x:.0f}mm y={y:.0f}mm theta={theta:.1f}deg")

        dist_to_home = np.hypot(x, y)
        if dist_to_home <= RETURN_HOME_TOLERANCE_MM:
            publish_speed_command(0.0, float("inf"), goal_reached=True)
            print(f"원점 복귀 완료 (남은 거리 {dist_to_home:.0f}mm).")
            return True

        min_obstacle_dist_mm = (float(np.min(np.hypot(current_scan[:, 0], current_scan[:, 1])))
                                 if len(current_scan) else float("inf"))
        speed_percent = 0.0 if min_obstacle_dist_mm < EMERGENCY_STOP_DIST_MM else CRUISE_SPEED_PERCENT_EXPLORE
        publish_speed_command(speed_percent, min_obstacle_dist_mm, goal_reached=False)

        start_rc = world_to_grid(x, y, origin, resolution)
        goal_rc = world_to_grid(0.0, 0.0, origin, resolution)
        path_rc = astar(inflated, start_rc, goal_rc)
        if path_rc is None:
            print("  복귀 경로를 못 찾았습니다. 다음 스캔에서 재시도합니다.")
            continue

        path_world = [grid_to_world(r, c, origin, resolution) for r, c in path_rc]
        steering_deg = pure_pursuit_steering(path_world, x, y, theta)
        publish_lidar_steering(steering_deg)

    print("[안내] 복귀 최대 사이클 도달 - 복귀를 완료하지 못했습니다.")
    return False


def main():
    parser = argparse.ArgumentParser(description="초기 위치만 지정하면 혼자 탐색하며 지도를 만들고 원점으로 복귀")
    parser.add_argument("--out", default="./map_output", help="완성된 맵을 저장할 폴더 (기본 ./map_output)")
    parser.add_argument("--max_cycles", type=int, default=MAX_EXPLORE_CYCLES,
                         help=f"탐색/복귀 각각의 최대 사이클 수 (기본 {MAX_EXPLORE_CYCLES}, 무한루프 방지용)")
    args = parser.parse_args()

    print("=== 실시간 라이다로 자율 탐색+매핑 시작 (Ctrl+C 종료) ===")
    ser, pwm, GPIO = open_lidar()

    def read_scan():
        return read_one_rotation(ser)

    try:
        map_points, grid, origin, resolution, pose = run_exploration(read_scan, args.max_cycles)
        export_map(args.out, map_points, grid, origin, resolution)
        print("\n=== 원점으로 복귀 시작 ===")
        return_to_origin(read_scan, map_points, grid, origin, resolution, pose, args.max_cycles)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        close_lidar(ser, pwm, GPIO)


if __name__ == "__main__":
    main()
