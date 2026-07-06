"""
localize_and_plan.py
======================
build_map.py 로 만든 맵(map_points.csv, map_occupancy_grid.npy, map_grid_meta.csv)을 읽어서
계속 반복하는 루프로 다음을 수행합니다:
1) 라이다로 스캔한 걸 가지고 "지금 내가 맵 좌표계에서 어디 있는지(x, y, theta)"를
   자동으로 찾아내고 (Localization)
2) 이번 스캔에서 보이는 장애물(정적 맵에 없던 것 포함)을 grid에 반영해서
3) 목표 지점까지 A* 또는 RRT*로 경로를 다시 계산하고 (장애물 회피 = 매 사이클 재계산)
4) 계획된 경로를 Pure Pursuit로 조향각으로 변환해서 lidar_steering.json 에 기록하고
5) 목표까지 남은 거리로 순항/정지 속도를 정해서 speed_command.json 에 기록합니다.

장애물 회피와 "어디로 갈지"의 판단은 전부 이 스크립트(Path_Planning)의 책임입니다.
Drive(Steer/Throttle)는 지도도 목적지도 모르는 실행 계층이라 회피 경로를 판단하지
않고, 여기서 내려주는 명령을 따르기만 합니다 (단, Drive 쪽에도 통신 두절/근접
장애물에 대비한 최후의 안전 정지 반사는 별도로 있습니다 - Drive/Throttle 참고).

조향은 Camera(차선 인식)와 여기(Pure Pursuit) 두 곳에서 각각 계산해서 서로 다른
파일(camera_steering.json / lidar_steering.json)에 기록하고, 최종적으로 어느 쪽을
쓸지는 Drive/Steer 가 두 값을 비교해서 결정합니다 (Steer/steer_control.py 참고).

방향(동서남북)을 몰라도 동작합니다 - 스캔의 점 패턴을 저장된 맵과 대조(Multi-start ICP)
해서 가장 잘 들어맞는 자세를 찾는 방식입니다. 매 사이클 맵 전체를 다시 뒤지면 너무
느리기 때문에, 직전 사이클에서 찾은 위치 근방만 좁게 재탐색하고(warm-start), 그
결과가 못 미더울 때만(오차가 크거나 첫 사이클) 맵 전체를 뒤지는 전체 탐색으로
전환합니다.

[사용법 - 실제 라이다로 계속 돌리는 모드 (라즈베리파이)]
    python3 localize_and_plan.py --map_dir ./map_output --goal 2000,1500

[사용법 - 이미 찍어둔 스캔 파일 하나로 한 번만 테스트해보는 모드]
    python3 localize_and_plan.py --map_dir ./map_output --scan current_scan.csv --goal 2000,1500 --once

--map_dir   : build_map.py 결과물(map_points.csv, map_occupancy_grid.npy, map_grid_meta.csv)이 있는 폴더
--goal      : 목표 위치, map 좌표계 기준 mm 단위 "x,y" (build_map.py가 만든 pos1 기준 좌표계).
              처음 보는 맵이라 목표 좌표를 모르겠으면 --scan --once 로 위치추정만 먼저 해보고
              map_result.png 를 보면서 목표 좌표를 정하면 됩니다.
--scan      : (선택) 실제 라이다 대신 이미 찍어둔 스캔 CSV 하나로 테스트. 생략하면 실제
              라이다(LD06)에서 계속 스캔을 읽어오는 루프로 동작합니다.
--once      : --scan 과 함께 써서 한 사이클만 실행하고 종료 (디버깅용)

[선택 옵션]
    --pos_candidates  최초(전체 탐색) 위치 후보 직접 지정 'x1,y1;x2,y2;...' (기본: 맵 전체 5x5 자동)
    --angle_step      전체 탐색 각도 간격 degree (기본 30도)
    --planner         astar(기본, 한 사이클에 ~수십ms) 또는 rrt(RRT*, 한 사이클에 수 초 - 반복 루프엔 비권장,
                       --once 테스트 용도로만 쓰는 걸 추천)
    --goal_tolerance  목표 도착으로 판정할 반경 mm (기본 200)

[출력 - Testing/state/ 에 기록, Drive 쪽이 읽어감]
    - speed_command.json   : 목표 속도(%) + 최근접 장애물 거리 + 목표 도착 여부
    - lidar_steering.json  : Pure Pursuit로 계산한 조향각
    - planned_path.csv/png : map_dir 안에 경로 저장 (--once 모드, 또는 --plot_every_n 지정시)
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

ICP_MAX_ITER = 40
ICP_TOLERANCE = 1e-5
ICP_MAX_CORRESPONDENCE_DIST = 300.0  # mm

ANGLE_UNIT = "deg"
DIST_UNIT = "mm"
MIN_RANGE_MM = 150
MAX_RANGE_MM = 8000

# Drive(Steer/Throttle)와 주고받는 상태 파일. Testing/ 폴더를 통째로 옮겨도 항상
# Testing/state 를 가리키도록 스크립트 위치 기준 상대경로로 계산
# (이 파일 위치: Testing/Path_Planning/localize_and_plan.py -> 한 단계 위가 Testing/)
TESTING_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATE_DIR = os.path.join(TESTING_DIR, "state")
SPEED_STATE_FILE = os.path.join(STATE_DIR, "speed_command.json")
LIDAR_STEERING_FILE = os.path.join(STATE_DIR, "lidar_steering.json")

CRUISE_SPEED_PERCENT = 40.0     # 장애물이 없을 때 순항 속도 (임시값, 실제 차량으로 튜닝 필요)
# [주의] 이 값은 반드시 (--inflate 기본값 * grid 해상도)보다 작아야 한다. A*가 짜는 정상 경로도
# inflate만큼은 벽에 붙어서 지나갈 수 있는데, 이 값이 그보다 크면 정상 경로 위에서조차 매 사이클
# 비상정지(speed=0)가 걸리고, 속도가 0이면 조향도 의미가 없어(다음 위치가 그대로) 제자리에 영원히
# 멈추는 교착상태에 빠진다 (Explore_Map 시뮬레이션에서 실제로 재현된 버그). 기본 --inflate=2,
# grid 해상도 50mm -> 여유거리 100mm 이므로 이보다 작게 잡음.
EMERGENCY_STOP_DIST_MM = 80.0
GOAL_REACH_TOLERANCE_MM = 200.0  # 이 거리 이내로 들어오면 "도착"으로 판정

# 위치추정 성능 관련. 맵 포인트가 수십만 개라 ICP 타겟으로 그대로 쓰면 매 사이클이
# 너무 느려짐 -> 위치추정용으로만 다운샘플한 사본을 따로 만들어 씀 (occupancy grid는 원본 그대로 사용)
MAP_ICP_DOWNSAMPLE = 20000

# 매 사이클 맵 전체(5x5 격자 x 여러 각도)를 다시 뒤지면 느리므로, 직전 위치 근방만 좁게
# 재탐색(warm-start)한다. 그 결과 오차가 너무 크면(=위치를 놓쳤을 가능성) 전체 재탐색으로 전환.
NARROW_POS_OFFSETS_MM = (-150.0, 0.0, 150.0)
NARROW_ANGLE_SPREAD_DEG = 15.0
RELOCALIZE_ERROR_THRESHOLD_MM = 150.0

WHEELBASE_MM = 150.0        # TODO: 실제 차량 축간거리(앞바퀴~뒷바퀴 축 간 거리)로 교체
LOOKAHEAD_MM = 400.0        # Pure Pursuit 전방주시거리 (튜닝 필요)
MAX_STEERING_DEG = 30.0     # Camera/Drive 쪽과 동일하게 맞춰야 함


# ----------------------------
# 스캔 로드 (build_map.py와 동일 로직)
# ----------------------------
def load_scan_as_xy(csv_path):
    df = pd.read_csv(csv_path)
    angle_col, dist_col = None, None
    for c in df.columns:
        cl = c.strip().lower()
        if cl in ("angle", "angle_deg", "angle_rad", "theta"):
            angle_col = c
        if cl in ("distance", "dist", "range", "distance_mm", "range_mm"):
            dist_col = c
    if angle_col is None or dist_col is None:
        raise ValueError(f"[{csv_path}] angle/distance 컬럼을 못 찾았어요. 실제 컬럼: {list(df.columns)}")

    angle = df[angle_col].to_numpy(dtype=float)
    dist = df[dist_col].to_numpy(dtype=float)
    return polar_to_xy(angle, dist)


def polar_to_xy(angle_deg, dist_mm):
    angle_rad = np.deg2rad(angle_deg) if ANGLE_UNIT == "deg" else angle_deg
    dist_mm = dist_mm * 1000.0 if DIST_UNIT == "m" else dist_mm
    valid = (dist_mm > MIN_RANGE_MM) & (dist_mm < MAX_RANGE_MM)
    angle_rad, dist_mm = angle_rad[valid], dist_mm[valid]
    x = dist_mm * np.cos(angle_rad)
    y = dist_mm * np.sin(angle_rad)
    return np.stack([x, y], axis=1)


def downsample_points(points, max_points):
    """점이 너무 많으면 ICP가 느려지므로 균일 샘플링 (build_map.py의 downsample()과 동일 로직)"""
    if len(points) <= max_points:
        return points
    idx = np.random.choice(len(points), max_points, replace=False)
    return points[idx]


# ----------------------------
# ICP (build_map.py와 동일 로직 + target KD-Tree 재사용 지원)
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
    """
    source 점군을 target 점군에 정합.
    init_guess: (dx, dy, dtheta_deg) - 대략적인 초기 이동/회전 추정치.
    target_tree: 미리 만들어둔 cKDTree(target). localize()에서 여러 후보를 시도할 때마다
    같은 target에 대해 매번 트리를 새로 만들면 낭비이므로, 호출부에서 한 번만 만들어 재사용.
    coarse-to-fine: 처음엔 넓은 범위로 매칭해서 큰 정렬을 잡고,
    반복할수록 max_corr_dist를 좁혀서 정밀도를 높임 (희소한 점군에서 미끄러짐 방지).
    반환: 정합된 source 좌표, 누적 R(2x2), 누적 t(2,), 평균오차
    """
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

    # coarse-to-fine: 1차로 넓게(예: 4x) 시작해서 점점 max_corr_dist까지 좁힘
    coarse_start = max_corr_dist * 4
    for i in range(max_iterations):
        progress = i / max(1, max_iterations - 1)
        current_max_corr = coarse_start + (max_corr_dist - coarse_start) * min(1.0, progress * 2)

        distances, indices = tree.query(src)
        mask = distances < current_max_corr
        if mask.sum() < 10:
            return src, R_total, t_total, 1e9  # 매칭 실패로 간주

        R, t = best_fit_transform(src[mask], target[indices[mask]])
        src = (R @ src.T).T + t
        R_total = R @ R_total
        t_total = R @ t_total + t

        # 수렴 판정은 최종(fine) 단계의 실제 오차로 함
        fine_mask = distances < max_corr_dist
        mean_error = np.mean(distances[fine_mask]) if fine_mask.sum() >= 10 else np.mean(distances[mask])
        if prev_error is not None and abs(prev_error - mean_error) < tolerance and progress >= 0.5:
            break
        prev_error = mean_error

    return src, R_total, t_total, mean_error


# ----------------------------
# 1단계: 현재 위치 찾기 (Multi-start ICP)
# ----------------------------
def localize(current_scan, full_map, pos_candidates=None, angle_step=30, angle_candidates=None):
    """
    current_scan: 지금 막 찍은 스캔 (Nx2, 라이다 자체 좌표계)
    full_map: 위치추정용 맵 포인트 (Mx2, map 좌표계). 호출부에서 이미 다운샘플된 것을 넘겨줌.
    pos_candidates: [(x,y), ...] 위치 탐색 후보. None이면 맵 영역을 5x5 격자로 자동 생성
    angle_candidates: 시도할 각도(deg) 리스트. None이면 0~360을 angle_step 간격으로 생성
    반환: (best_x, best_y, best_theta_deg, best_err)
    """
    if pos_candidates is None:
        min_x, max_x = full_map[:, 0].min(), full_map[:, 0].max()
        min_y, max_y = full_map[:, 1].min(), full_map[:, 1].max()
        xs = np.linspace(min_x, max_x, 5)
        ys = np.linspace(min_y, max_y, 5)
        pos_candidates = [(x, y) for x in xs for y in ys]
    if angle_candidates is None:
        angle_candidates = list(range(0, 360, angle_step))

    print(f"  위치 후보 {len(pos_candidates)}개 x 각도 후보 {len(angle_candidates)}개 = "
          f"{len(pos_candidates) * len(angle_candidates)}번 ICP 시도 중...")

    target_tree = cKDTree(full_map)  # 이번 localize() 호출 안에서 모든 후보가 공유해서 재사용
    best = None  # (err, x, y, theta)
    for px, py in pos_candidates:
        for theta0 in angle_candidates:
            _, R, t, err = icp(current_scan, full_map, init_guess=(px, py, theta0), target_tree=target_tree)
            theta = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
            if best is None or err < best[0]:
                best = (err, t[0], t[1], theta)

    err, x, y, theta = best
    print(f"  최적 후보: x={x:.0f}mm, y={y:.0f}mm, theta={theta:.1f}deg (정합 오차={err:.1f}mm)")
    if err > 100:
        print("  [주의] 정합 오차가 큽니다 (100mm 초과). 위치 추정이 부정확할 수 있어요.")
        print("         - 방에 가구/모서리 등 특징물이 적은 경우 흔히 발생합니다.")
        print("         - 스캔 위치를 약간 옮기거나, --pos_candidates로 대략적 위치를 직접 좁혀서 재시도해보세요.")
    return x, y, theta, err


def localize_warmstart(current_scan, map_points_icp, prev_pose, pos_candidates, angle_step):
    """
    직전 사이클에서 찾은 위치(prev_pose)가 있으면 그 근방만 좁게 재탐색 (훨씬 빠름).
    오차가 RELOCALIZE_ERROR_THRESHOLD_MM보다 크면(=위치를 놓쳤을 가능성) 맵 전체 재탐색으로 폴백.
    반환: (x, y, theta, err, mode) - mode는 "narrow" 또는 "wide" (로그/디버깅용)
    """
    if prev_pose is not None:
        px, py, ptheta = prev_pose
        narrow_pos = [(px + dx, py + dy) for dx in NARROW_POS_OFFSETS_MM for dy in NARROW_POS_OFFSETS_MM]
        narrow_angles = [(ptheta + d) % 360 for d in (-NARROW_ANGLE_SPREAD_DEG, 0.0, NARROW_ANGLE_SPREAD_DEG)]
        x, y, theta, err = localize(current_scan, map_points_icp, pos_candidates=narrow_pos,
                                     angle_candidates=narrow_angles)
        if err <= RELOCALIZE_ERROR_THRESHOLD_MM:
            return x, y, theta, err, "narrow"
        print("  [재탐색] 좁은 범위 탐색 오차가 커서 맵 전체 재탐색으로 전환합니다 (위치를 놓쳤을 수 있음)")

    x, y, theta, err = localize(current_scan, map_points_icp, pos_candidates=pos_candidates, angle_step=angle_step)
    return x, y, theta, err, "wide"


# ----------------------------
# 2단계: 격자 변환 + 장애물
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
    """장애물 주변을 약간 부풀려서 로봇이 벽에 너무 붙어서 가지 않도록 함"""
    from scipy.ndimage import binary_dilation
    return binary_dilation(grid, iterations=inflate_cells).astype(np.uint8)


# ----------------------------
# 실시간 장애물 회피
# 장애물 회피는 여기(Path_Planning)에서 "경로를 다시 계산"하는 방식으로 처리한다.
# Drive(Steer/Throttle)는 지도도 목적지도 모르는 실행 계층이라 회피 경로를 판단할 수 없고,
# 여기서 내린 결정을 그대로 따르기만 한다. 대신 Drive 쪽엔 통신이 끊기거나 뭔가 아주 가까이
# 붙었을 때를 대비한 최후의 안전 반사(정지)만 별도로 둔다 (Drive/Throttle/throttle_control.py 참고).
# ----------------------------
def transform_points(points, x, y, theta_deg):
    """라이다(로봇) 좌표계의 점들을 localize()가 찾은 (x,y,theta)로 map 좌표계로 변환"""
    th = np.deg2rad(theta_deg)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return (R @ points.T).T + np.array([x, y])


def inject_scan_obstacles(grid, origin, resolution, scan_world_xy):
    """
    이번에 실시간으로 찍은 스캔에 찍힌 점들을 grid에 장애물로 추가 반영.
    build_map.py 로 만든 정적 맵에는 없던 새 장애물(사람, 상자 등)도
    이번 스캔 기준으로 즉시 경로계획에서 회피 대상이 되도록 하기 위함.
    """
    updated = grid.copy()
    h, w = updated.shape
    for x_mm, y_mm in scan_world_xy:
        row, col = world_to_grid(x_mm, y_mm, origin, resolution)
        if 0 <= row < h and 0 <= col < w:
            updated[row, col] = 1
    return updated


def publish_speed_command(speed_percent, min_obstacle_dist_mm, goal_reached=False):
    """Drive/Throttle 쪽이 읽어가는 목표 속도 명령을 기록"""
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp_path = SPEED_STATE_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump({
            "speed_percent": speed_percent,
            "min_obstacle_dist_mm": min_obstacle_dist_mm,
            "goal_reached": goal_reached,
            "timestamp": time.time(),
        }, f)
    os.replace(tmp_path, SPEED_STATE_FILE)  # 원자적 교체: Throttle이 쓰다 만 파일을 읽지 않도록 함


def publish_lidar_steering(angle_deg):
    """Drive/Steer 쪽이 camera_steering.json과 비교해서 최종 조향을 정할 때 쓰는 값"""
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp_path = LIDAR_STEERING_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump({"angle_deg": angle_deg, "timestamp": time.time()}, f)
    os.replace(tmp_path, LIDAR_STEERING_FILE)


# ----------------------------
# 3단계: 경로 -> 조향각 (Pure Pursuit)
# ----------------------------
def pure_pursuit_steering(path_world, x, y, theta_deg, lookahead_mm=LOOKAHEAD_MM,
                           wheelbase_mm=WHEELBASE_MM, max_steering_deg=MAX_STEERING_DEG):
    """
    계획된 경로(path_world)와 현재 위치/자세로 목표 조향각을 계산 (Ackermann Pure Pursuit).
    WHEELBASE_MM은 실제 차량 축간거리로 나중에 교체해야 하는 임시값.
    """
    path = np.asarray(path_world, dtype=float)
    if len(path) == 0:
        return 0.0

    dists = np.hypot(path[:, 0] - x, path[:, 1] - y)
    ahead_idx = np.where(dists >= lookahead_mm)[0]
    target = path[ahead_idx[0]] if len(ahead_idx) else path[-1]

    target_heading_deg = np.degrees(np.arctan2(target[1] - y, target[0] - x))
    alpha_deg = (target_heading_deg - theta_deg + 180) % 360 - 180  # -180~180로 정규화
    alpha = np.deg2rad(alpha_deg)

    actual_lookahead = max(1.0, np.hypot(target[0] - x, target[1] - y))
    curvature = 2.0 * np.sin(alpha) / actual_lookahead
    steering_deg = np.degrees(np.arctan(wheelbase_mm * curvature))
    return max(-max_steering_deg, min(max_steering_deg, steering_deg))


def is_segment_free(grid, origin, resolution, p1, p2):
    """world 좌표 두 점을 잇는 직선이 장애물(inflated grid)을 지나지 않는지 확인"""
    h, w = grid.shape
    dist = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
    steps = max(2, int(dist / (resolution / 2)))
    for i in range(steps + 1):
        t = i / steps
        x = p1[0] + (p2[0] - p1[0]) * t
        y = p1[1] + (p2[1] - p1[1]) * t
        row, col = world_to_grid(x, y, origin, resolution)
        if not (0 <= row < h and 0 <= col < w):
            return False
        if grid[row, col] == 1:
            return False
    return True


def rrt_star(grid, origin, resolution, start, goal, max_iter=3000, step_size=150.0,
             goal_sample_rate=0.1, search_radius=300.0, goal_tolerance=100.0):
    """
    RRT*: world 좌표(mm) 상에서 직접 트리를 뻗어가며 경로를 탐색.
    grid: inflate_obstacles() 를 거친 occupancy grid (0=빈공간, 1=장애물)
    start/goal: (x_mm, y_mm)
    반환: world 좌표 경로 [(x,y), ...] 또는 목표에 도달 못하면 None

    참고: 기본 5000 iter 기준 실측 약 3~6초/사이클(개발 PC 기준, 라즈베리파이는 더 느릴 가능성)
    이라 매 사이클 다시 계산해야 하는 실시간 루프에는 비권장. --once 테스트나 --planner astar로
    막힌 좁은 통로 보정 용도로만 쓰는 걸 추천.
    """
    height, width = grid.shape
    min_x, min_y = origin
    max_x, max_y = min_x + width * resolution, min_y + height * resolution

    nodes = [start]
    parent = {0: None}
    cost = {0: 0.0}
    best_goal_idx, best_goal_cost = None, float("inf")

    for _ in range(max_iter):
        rnd = goal if np.random.rand() < goal_sample_rate else (
            np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y))

        nearest_idx = min(range(len(nodes)),
                           key=lambda i: np.hypot(nodes[i][0] - rnd[0], nodes[i][1] - rnd[1]))
        nearest_pt = nodes[nearest_idx]
        d = np.hypot(rnd[0] - nearest_pt[0], rnd[1] - nearest_pt[1])
        if d <= step_size:
            new_pt = rnd
        else:
            ratio = step_size / d
            new_pt = (nearest_pt[0] + (rnd[0] - nearest_pt[0]) * ratio,
                      nearest_pt[1] + (rnd[1] - nearest_pt[1]) * ratio)

        if not is_segment_free(grid, origin, resolution, nearest_pt, new_pt):
            continue

        near_idxs = [i for i, n in enumerate(nodes)
                     if np.hypot(n[0] - new_pt[0], n[1] - new_pt[1]) <= search_radius]

        best_parent, best_cost_new = nearest_idx, cost[nearest_idx] + d
        for i in near_idxs:
            c = cost[i] + np.hypot(nodes[i][0] - new_pt[0], nodes[i][1] - new_pt[1])
            if c < best_cost_new and is_segment_free(grid, origin, resolution, nodes[i], new_pt):
                best_parent, best_cost_new = i, c

        new_idx = len(nodes)
        nodes.append(new_pt)
        parent[new_idx] = best_parent
        cost[new_idx] = best_cost_new

        for i in near_idxs:
            c = best_cost_new + np.hypot(new_pt[0] - nodes[i][0], new_pt[1] - nodes[i][1])
            if c < cost[i] and is_segment_free(grid, origin, resolution, new_pt, nodes[i]):
                parent[i] = new_idx
                cost[i] = c

        d_to_goal = np.hypot(new_pt[0] - goal[0], new_pt[1] - goal[1])
        if d_to_goal <= goal_tolerance:
            total = cost[new_idx] + d_to_goal
            if total < best_goal_cost and is_segment_free(grid, origin, resolution, new_pt, goal):
                best_goal_cost, best_goal_idx = total, new_idx

    if best_goal_idx is None:
        return None

    path = [goal]
    idx = best_goal_idx
    while idx is not None:
        path.append(nodes[idx])
        idx = parent[idx]
    path.reverse()
    return path


def astar(grid, start, goal):
    """grid: 0=빈공간, 1=장애물. start/goal: (row, col). 실측 약 60ms/사이클(65x75 grid 기준)"""
    h, w = grid.shape
    if grid[start] == 1:
        print("  [경고] 시작 위치가 장애물 칸 위에 있습니다. inflate를 줄이거나 위치를 확인하세요.")
    if grid[goal] == 1:
        print("  [경고] 목표 위치가 장애물 칸 위에 있습니다.")

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
    return None  # 경로 없음


# ----------------------------
# 실시간 라이다(LD06) 읽기 - Build_Map/ld06_test.py 와 동일한 패킷 파싱
# 여기선 CSV로 저장하지 않고 바로 메모리에서 좌표로 변환해서 씀
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
    """대략 한 바퀴(360도) 분량의 (angle_deg, distance_mm) 점을 모아서 반환.
    max_wait_s 를 넘기면(신호 불안정 등) 그때까지 모은 것만 반환 (무한 대기 방지)"""
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


def scan_points_to_xy(points):
    """[(angle_deg, distance_mm), ...] -> Nx2 xy (load_scan_as_xy와 동일한 필터링/좌표계)"""
    if not points:
        return np.empty((0, 2))
    arr = np.array(points, dtype=float)
    return polar_to_xy(arr[:, 0], arr[:, 1])


# ----------------------------
# 한 사이클: 위치추정 -> 장애물 반영 -> 경로계획 -> 조향/속도 명령 발행
# ----------------------------
def run_cycle(current_scan, map_points_icp, static_grid, origin, resolution, goal_xy,
              prev_pose, args, save_debug_files=False):
    """
    한 번의 스캔으로 위치추정부터 명령 발행까지 전부 수행.
    반환: (x, y, theta, err, path_world 또는 None, arrived: bool)
    """
    pos_candidates = args.pos_candidates_parsed
    x, y, theta, err, mode = localize_warmstart(current_scan, map_points_icp, prev_pose,
                                                 pos_candidates, args.angle_step)
    print(f"  >>> 위치({mode}): x={x:.0f}mm, y={y:.0f}mm, heading={theta:.1f}deg, 오차={err:.1f}mm")

    scan_world = transform_points(current_scan, x, y, theta)
    grid = inject_scan_obstacles(static_grid, origin, resolution, scan_world)
    min_obstacle_dist_mm = (float(np.min(np.hypot(current_scan[:, 0], current_scan[:, 1])))
                             if len(current_scan) else float("inf"))

    goal_x, goal_y = goal_xy
    dist_to_goal = np.hypot(goal_x - x, goal_y - y)
    arrived = dist_to_goal <= args.goal_tolerance

    if arrived:
        publish_speed_command(0.0, min_obstacle_dist_mm, goal_reached=True)
        print(f"  [도착] 목표까지 {dist_to_goal:.0f}mm - 목표 반경({args.goal_tolerance:.0f}mm) 이내라 정지 명령을 보냈습니다.")
        return x, y, theta, err, None, True

    speed_percent = 0.0 if min_obstacle_dist_mm < EMERGENCY_STOP_DIST_MM else CRUISE_SPEED_PERCENT
    publish_speed_command(speed_percent, min_obstacle_dist_mm, goal_reached=False)
    if speed_percent == 0.0:
        print(f"  [비상정지] 장애물이 {EMERGENCY_STOP_DIST_MM:.0f}mm 이내라 Drive/Throttle에 정지 명령을 보냈습니다.")

    inflated = inflate_obstacles(grid, args.inflate)
    start_rc = world_to_grid(x, y, origin, resolution)
    goal_rc = world_to_grid(goal_x, goal_y, origin, resolution)

    if args.planner == "astar":
        path_rc = astar(inflated, start_rc, goal_rc)
        path_world = [grid_to_world(r, c, origin, resolution) for r, c in path_rc] if path_rc else None
    else:
        path_world = rrt_star(inflated, origin, resolution, (x, y), (goal_x, goal_y),
                               max_iter=args.rrt_iter, step_size=args.rrt_step,
                               goal_tolerance=args.rrt_goal_tolerance)
        path_rc = [world_to_grid(px, py, origin, resolution) for px, py in path_world] if path_world else None

    if path_world is None:
        print("  경로를 찾지 못했습니다. 이번 사이클은 조향각을 갱신하지 않습니다.")
        return x, y, theta, err, None, False

    steering_deg = pure_pursuit_steering(path_world, x, y, theta)
    publish_lidar_steering(steering_deg)
    print(f"  경로 {len(path_world)}개 waypoint, Pure Pursuit 조향각: {steering_deg:+.1f}deg")

    if save_debug_files:
        save_path_debug_files(args.map_dir, grid, path_rc, path_world, start_rc, goal_rc, args.planner)

    return x, y, theta, err, path_world, False


def save_path_debug_files(map_dir, grid, path_rc, path_world, start_rc, goal_rc, planner_name):
    out_csv = os.path.join(map_dir, "planned_path.csv")
    pd.DataFrame(path_world, columns=["x_mm", "y_mm"]).to_csv(out_csv, index=False)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap="gray_r", origin="upper")
    path_arr = np.array(path_rc)
    ax.plot(path_arr[:, 1], path_arr[:, 0], color="red", linewidth=2, label="planned path")
    ax.scatter([start_rc[1]], [start_rc[0]], color="blue", s=80, marker="o", label="start (current pos)")
    ax.scatter([goal_rc[1]], [goal_rc[0]], color="green", s=80, marker="*", label="goal")
    ax.legend()
    ax.set_title(f"{planner_name.upper()} Path Plan")
    out_png = os.path.join(map_dir, "planned_path.png")
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    print(f"  경로 저장: {out_csv}, {out_png}")


# ----------------------------
# 메인
# ----------------------------
def main():
    parser = argparse.ArgumentParser(description="저장된 맵에서 현재 위치를 찾고 목표까지 계속 경로를 계획/추종")
    parser.add_argument("--map_dir", required=True, help="build_map.py 결과물이 있는 폴더")
    parser.add_argument("--goal", required=True, help="목표 위치 'x_mm,y_mm' (map 좌표계)")
    parser.add_argument("--scan", default=None,
                         help="(선택) 실제 라이다 대신 이 스캔 CSV 하나로 테스트. 생략하면 실제 라이다 루프로 동작")
    parser.add_argument("--once", action="store_true", help="--scan 과 함께 한 사이클만 실행하고 종료 (디버깅용)")
    parser.add_argument("--pos_candidates", default=None,
                         help="최초(전체 탐색) 위치 후보 직접 지정 'x1,y1;x2,y2;...' (생략시 맵 전체 5x5 자동)")
    parser.add_argument("--angle_step", type=int, default=30, help="전체 탐색 각도 간격 degree (기본 30)")
    parser.add_argument("--inflate", type=int, default=2,
                         help="장애물 팽창 칸 수, 로봇 크기 여유 (기본 2칸). "
                              "주의: inflate*grid해상도(mm)는 반드시 EMERGENCY_STOP_DIST_MM보다 커야 함 - "
                              "안 그러면 정상 경로 위에서도 raw 스캔 최근접거리가 계속 그 이내로 잡혀서 "
                              "매 사이클 비상정지가 걸리고 회전도 안 되어(v=0) 제자리에 멈춰버리는 "
                              "교착상태에 빠짐 (Explore_Map 시뮬레이션에서 실제로 재현된 버그). "
                              "실제 방 크기 기준 inflate를 늘리면 통로 자체가 막혀버릴 수 있어서(실측 확인됨) "
                              "이 값을 올리기보다는 EMERGENCY_STOP_DIST_MM을 낮추는 쪽으로 맞춰뒀음")
    parser.add_argument("--planner", choices=["astar", "rrt"], default="astar",
                         help="경로계획 알고리즘: astar(기본, 실시간 루프용) 또는 rrt(RRT*, 느림 - --once 용)")
    parser.add_argument("--rrt_iter", type=int, default=5000, help="RRT* 반복 횟수 (기본 5000)")
    parser.add_argument("--rrt_step", type=float, default=150.0, help="RRT* 한 스텝 거리 mm (기본 150)")
    parser.add_argument("--rrt_goal_tolerance", type=float, default=150.0,
                         help="RRT*가 목표에 도달했다고 볼 반경 mm (기본 150)")
    parser.add_argument("--goal_tolerance", type=float, default=GOAL_REACH_TOLERANCE_MM,
                         help=f"목표 도착으로 판정할 반경 mm (기본 {GOAL_REACH_TOLERANCE_MM:.0f})")
    args = parser.parse_args()

    if args.planner == "rrt" and not args.once:
        print("[안내] --planner rrt 는 사이클당 수 초가 걸려 실시간 루프엔 느립니다. "
              "--once 테스트가 아니면 astar를 권장합니다.")

    args.pos_candidates_parsed = None
    if args.pos_candidates:
        args.pos_candidates_parsed = [tuple(map(float, pair.split(","))) for pair in args.pos_candidates.split(";")]

    goal_xy = tuple(map(float, args.goal.split(",")))

    print("=== 맵 로드 ===")
    map_points = pd.read_csv(os.path.join(args.map_dir, "map_points.csv"))[["x_mm", "y_mm"]].to_numpy()
    static_grid = np.load(os.path.join(args.map_dir, "map_occupancy_grid.npy"))
    meta = pd.read_csv(os.path.join(args.map_dir, "map_grid_meta.csv")).iloc[0]
    origin = (meta["origin_x_mm"], meta["origin_y_mm"])
    resolution = meta["resolution_mm"]
    map_points_icp = downsample_points(map_points, MAP_ICP_DOWNSAMPLE)
    print(f"  맵 포인트 {len(map_points)}개 (위치추정용 {len(map_points_icp)}개로 다운샘플), grid shape={static_grid.shape}")

    inflate_clearance_mm = args.inflate * resolution
    if inflate_clearance_mm <= EMERGENCY_STOP_DIST_MM:
        print(f"  [경고] --inflate 여유거리({inflate_clearance_mm:.0f}mm)가 EMERGENCY_STOP_DIST_MM"
              f"({EMERGENCY_STOP_DIST_MM:.0f}mm)보다 작거나 같습니다. 정상 경로 위에서도 비상정지가 "
              f"계속 걸려 제자리에 멈추는 교착상태에 빠질 수 있어요. --inflate를 "
              f"{int(EMERGENCY_STOP_DIST_MM / resolution) + 1} 이상으로 늘리는 걸 권장합니다.")

    if args.scan:
        current_scan = load_scan_as_xy(args.scan)
        print(f"\n=== 스캔 파일 로드: {len(current_scan)}개 포인트 ===")
        prev_pose = None
        while True:
            x, y, theta, err, path_world, arrived = run_cycle(
                current_scan, map_points_icp, static_grid, origin, resolution, goal_xy,
                prev_pose, args, save_debug_files=True)
            prev_pose = (x, y, theta)
            if args.once or arrived:
                break
        return

    print("\n=== 실시간 라이다 루프 시작 (Ctrl+C 종료) ===")
    ser, pwm, GPIO = open_lidar()
    prev_pose = None
    try:
        while True:
            cycle_t0 = time.time()
            scan_points = read_one_rotation(ser)
            current_scan = scan_points_to_xy(scan_points)
            if len(current_scan) < 10:
                print("  스캔 포인트가 너무 적습니다. 다음 회전을 기다립니다.")
                continue

            x, y, theta, err, path_world, arrived = run_cycle(
                current_scan, map_points_icp, static_grid, origin, resolution, goal_xy,
                prev_pose, args, save_debug_files=False)
            prev_pose = (x, y, theta)
            print(f"  사이클 시간: {time.time() - cycle_t0:.2f}s")

            if arrived:
                print("목표에 도착했습니다. 종료합니다.")
                break
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        close_lidar(ser, pwm, GPIO)


if __name__ == "__main__":
    main()
