import json
import struct

import numpy as np
import pytest

import localize_and_plan as pp


# ----------------------------
# grid <-> world
# ----------------------------
def test_world_to_grid_grid_to_world_roundtrip():
    origin = (-1000.0, -500.0)
    resolution = 50.0
    for row in range(0, 40, 3):
        for col in range(0, 40, 3):
            x, y = pp.grid_to_world(row, col, origin, resolution)
            r2, c2 = pp.world_to_grid(x, y, origin, resolution)
            assert (r2, c2) == (row, col)


def test_world_to_grid_origin_maps_to_zero():
    origin = (100.0, 200.0)
    resolution = 50.0
    assert pp.world_to_grid(100.0, 200.0, origin, resolution) == (0, 0)


# ----------------------------
# transform_points
# ----------------------------
def test_transform_points_identity():
    pts = np.array([[10.0, 20.0], [30.0, -5.0]])
    out = pp.transform_points(pts, 0.0, 0.0, 0.0)
    assert np.allclose(out, pts)


def test_transform_points_translation_only():
    pts = np.array([[0.0, 0.0]])
    out = pp.transform_points(pts, 100.0, 200.0, 0.0)
    assert np.allclose(out, [[100.0, 200.0]])


def test_transform_points_rotation_90():
    pts = np.array([[100.0, 0.0]])
    out = pp.transform_points(pts, 0.0, 0.0, 90.0)
    # rotating (100,0) by +90deg -> (0,100)
    assert np.allclose(out, [[0.0, 100.0]], atol=1e-9)


# ----------------------------
# compute_min_obstacle_dist_mm
# ----------------------------
def test_compute_min_obstacle_dist_mm_basic():
    scan = np.array([[300.0, 0.0], [0.0, 100.0], [50.0, 0.0]])
    assert pp.compute_min_obstacle_dist_mm(scan) == pytest.approx(50.0)


def test_compute_min_obstacle_dist_mm_empty():
    scan = np.empty((0, 2))
    assert pp.compute_min_obstacle_dist_mm(scan) == float("inf")


# ----------------------------
# predict_closing_stop
# ----------------------------
def test_predict_closing_stop_no_prev():
    should_stop, speed = pp.predict_closing_stop(500.0, None, 0.1)
    assert should_stop is False
    assert speed == 0.0


def test_predict_closing_stop_zero_dt():
    should_stop, speed = pp.predict_closing_stop(500.0, 600.0, 0.0)
    assert should_stop is False


def test_predict_closing_stop_slow_closing_ignored():
    # closing speed = (600-590)/1.0 = 10mm/s < MIN_CLOSING_SPEED_FOR_PREDICTION_MM_S
    should_stop, speed = pp.predict_closing_stop(590.0, 600.0, 1.0)
    assert should_stop is False
    assert speed == pytest.approx(10.0)


def test_predict_closing_stop_fast_closing_triggers():
    # dist=100mm, closing speed=1000mm/s -> ttc=0.1s < PREDICTIVE_STOP_TTC_S(1.0)
    should_stop, speed = pp.predict_closing_stop(100.0, 1100.0, 1.0)
    assert should_stop is True
    assert speed == pytest.approx(1000.0)


def test_predict_closing_stop_fast_but_far_no_trigger():
    # closing speed=1000mm/s (>=threshold) but dist so large ttc>1s
    should_stop, speed = pp.predict_closing_stop(5000.0, 6000.0, 1.0)
    assert should_stop is False
    assert speed == pytest.approx(1000.0)


def test_predict_closing_stop_receding_obstacle_ignored():
    # obstacle getting farther away -> negative closing speed -> never stop
    should_stop, speed = pp.predict_closing_stop(700.0, 600.0, 1.0)
    assert should_stop is False
    assert speed < 0


# ----------------------------
# inject_scan_obstacles / clear_confirmed_free_cells
# ----------------------------
def test_inject_scan_obstacles_marks_cell():
    grid = np.zeros((10, 10), dtype=np.uint8)
    origin = (0.0, 0.0)
    resolution = 10.0
    scan_world = np.array([[35.0, 45.0]])  # -> row=4, col=3
    updated = pp.inject_scan_obstacles(grid, origin, resolution, scan_world)
    assert updated[4, 3] == 1
    assert grid[4, 3] == 0  # original grid untouched (copy semantics)


def test_inject_scan_obstacles_out_of_bounds_ignored():
    grid = np.zeros((5, 5), dtype=np.uint8)
    origin = (0.0, 0.0)
    resolution = 10.0
    scan_world = np.array([[9999.0, 9999.0]])
    updated = pp.inject_scan_obstacles(grid, origin, resolution, scan_world)
    assert updated.sum() == 0


def test_clear_confirmed_free_cells_clears_path_but_not_endpoint():
    grid = np.ones((20, 20), dtype=np.uint8)  # everything obstacle initially
    origin = (0.0, 0.0)
    resolution = 10.0
    robot_xy = (5.0, 5.0)
    scan_world = np.array([[155.0, 5.0]])  # straight line along x
    updated = pp.clear_confirmed_free_cells(grid, origin, resolution, robot_xy, scan_world)
    # cell right next to the robot along the ray should be cleared
    r0, c0 = pp.world_to_grid(15.0, 5.0, origin, resolution)
    assert updated[r0, c0] == 0
    # the endpoint cell itself must remain untouched (still obstacle)
    r_end, c_end = pp.world_to_grid(155.0, 5.0, origin, resolution)
    assert updated[r_end, c_end] == 1


# ----------------------------
# pure_pursuit_steering
# ----------------------------
def test_pure_pursuit_steering_straight_path_is_zero():
    path = [(0.0, 0.0), (100.0, 0.0), (1000.0, 0.0)]
    steer = pp.pure_pursuit_steering(path, 0.0, 0.0, 0.0)
    assert steer == pytest.approx(0.0, abs=1e-6)


def test_pure_pursuit_steering_empty_path():
    assert pp.pure_pursuit_steering([], 0.0, 0.0, 0.0) == 0.0


def test_pure_pursuit_steering_left_turn_positive():
    # heading 0 (facing +x), path curves toward +y (to the left) -> steering should be positive
    path = [(0.0, 0.0), (200.0, 200.0), (500.0, 500.0)]
    steer = pp.pure_pursuit_steering(path, 0.0, 0.0, 0.0)
    assert steer > 0


def test_pure_pursuit_steering_right_turn_negative():
    path = [(0.0, 0.0), (200.0, -200.0), (500.0, -500.0)]
    steer = pp.pure_pursuit_steering(path, 0.0, 0.0, 0.0)
    assert steer < 0


def test_pure_pursuit_steering_clamped_to_max():
    # target directly behind/sideways -> large alpha -> should clamp
    path = [(0.0, 500.0)]  # directly to the left, heading 0
    steer = pp.pure_pursuit_steering(path, 0.0, 0.0, 0.0, max_steering_deg=30.0)
    assert abs(steer) <= 30.0 + 1e-9


# ----------------------------
# curvature_speed_factor
# ----------------------------
def test_curvature_speed_factor_zero_steering():
    assert pp.curvature_speed_factor(0.0) == pytest.approx(1.0)


def test_curvature_speed_factor_max_steering():
    assert pp.curvature_speed_factor(pp.MAX_STEERING_DEG) == pytest.approx(pp.CURVE_SPEED_MIN_FACTOR)


def test_curvature_speed_factor_beyond_max_clamped():
    # steering beyond max_steering_deg shouldn't push factor below the floor
    assert pp.curvature_speed_factor(pp.MAX_STEERING_DEG * 5) == pytest.approx(pp.CURVE_SPEED_MIN_FACTOR)


def test_curvature_speed_factor_negative_steering_symmetric():
    assert pp.curvature_speed_factor(-15.0) == pp.curvature_speed_factor(15.0)


# ----------------------------
# is_segment_free
# ----------------------------
def test_is_segment_free_true_when_clear():
    grid = np.zeros((10, 10), dtype=np.uint8)
    origin = (0.0, 0.0)
    resolution = 10.0
    assert pp.is_segment_free(grid, origin, resolution, (5.0, 5.0), (95.0, 5.0)) is True


def test_is_segment_free_false_when_blocked():
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[0, 5] = 1  # somewhere along the horizontal ray at y=5,row=0
    origin = (0.0, 0.0)
    resolution = 10.0
    assert pp.is_segment_free(grid, origin, resolution, (5.0, 5.0), (95.0, 5.0)) is False


def test_is_segment_free_out_of_bounds():
    grid = np.zeros((5, 5), dtype=np.uint8)
    origin = (0.0, 0.0)
    resolution = 10.0
    assert pp.is_segment_free(grid, origin, resolution, (5.0, 5.0), (9999.0, 5.0)) is False


# ----------------------------
# astar
# ----------------------------
def test_astar_finds_path_in_open_grid():
    grid = np.zeros((10, 10), dtype=np.uint8)
    path = pp.astar(grid, (0, 0), (9, 9))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (9, 9)


def test_astar_returns_none_when_goal_unreachable():
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[:, 5] = 1  # solid wall splitting the grid
    path = pp.astar(grid, (0, 0), (0, 9))
    assert path is None


def test_astar_start_out_of_bounds_returns_none():
    grid = np.zeros((5, 5), dtype=np.uint8)
    assert pp.astar(grid, (-1, 0), (4, 4)) is None


def test_astar_goal_out_of_bounds_returns_none():
    grid = np.zeros((5, 5), dtype=np.uint8)
    assert pp.astar(grid, (0, 0), (5, 5)) is None


def test_astar_start_equals_goal():
    grid = np.zeros((5, 5), dtype=np.uint8)
    path = pp.astar(grid, (2, 2), (2, 2))
    assert path == [(2, 2)]


# ----------------------------
# ICP
# ----------------------------
def _make_wall_cloud(n=400, seed=0):
    rng = np.random.default_rng(seed)
    # an L-shaped "room" so ICP has enough geometric structure to converge
    xs = np.linspace(-1000, 1000, n // 2)
    wall1 = np.stack([xs, np.full_like(xs, 1000.0)], axis=1)
    ys = np.linspace(-1000, 1000, n // 2)
    wall2 = np.stack([np.full_like(ys, 1000.0), ys], axis=1)
    return np.vstack([wall1, wall2])


def test_icp_recovers_known_transform():
    target = _make_wall_cloud()
    true_dx, true_dy, true_theta = 120.0, -80.0, 10.0
    th = np.deg2rad(true_theta)
    Rt = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    t = np.array([true_dx, true_dy])
    # target = Rt @ source + t  =>  source = Rt^-1 @ (target - t)
    source = (target - t) @ Rt  # Rt^-1 == Rt.T, and (A @ Rt.T) == (Rt @ A.T).T style; using row-vector convention

    _, R_est, t_est, err = pp.icp(source, target, init_guess=(0.0, 0.0, 0.0))
    est_theta = np.degrees(np.arctan2(R_est[1, 0], R_est[0, 0]))

    assert err < 20.0  # mm, should converge tightly on this clean synthetic data
    assert est_theta == pytest.approx(true_theta, abs=2.0)
    assert t_est[0] == pytest.approx(true_dx, abs=20.0)
    assert t_est[1] == pytest.approx(true_dy, abs=20.0)


def test_icp_too_few_points_reports_failure():
    target = _make_wall_cloud()
    source = target[:3]  # far too few points to match
    _, _, _, err = pp.icp(source, target)
    assert err == 1e9


# ----------------------------
# ld06 parsing helpers
# ----------------------------
def test_ld06_point_angles_no_wrap():
    angles = pp.ld06_point_angles(10.0, 20.0, 3)
    assert angles == pytest.approx([10.0, 15.0, 20.0])


def test_ld06_point_angles_wraps_past_360():
    angles = pp.ld06_point_angles(350.0, 10.0, 3)
    assert angles[0] == pytest.approx(350.0)
    assert angles[-1] == pytest.approx(10.0)
    # midpoint should be 0 (i.e. 360 % 360)
    assert angles[1] == pytest.approx(0.0, abs=1e-9)


def test_ld06_point_angles_single_point_no_div_by_zero():
    angles = pp.ld06_point_angles(10.0, 20.0, 1)
    assert angles == [10.0]


def _build_ld06_packet(start_angle_deg, end_angle_deg, distances):
    assert len(distances) == 12
    header = bytes([0x54, 0x2C])
    speed = struct.pack('<H', 3000)  # 30.00 deg/unit *100
    start = struct.pack('<H', int(start_angle_deg * 100))
    body = b"".join(struct.pack('<H', d) + bytes([200]) for d in distances)
    end = struct.pack('<H', int(end_angle_deg * 100))
    timestamp = struct.pack('<H', 0)
    crc = bytes([0])  # real LD06 packets have a trailing CRC byte; parse_ld06_packet ignores it
    packet = header + speed + start + body + end + timestamp + crc
    assert len(packet) == 47
    return packet


def test_parse_ld06_packet_valid():
    packet = _build_ld06_packet(0.0, 30.0, [500] * 12)
    result = pp.parse_ld06_packet(packet)
    assert result is not None
    assert result["start_angle"] == pytest.approx(0.0)
    assert result["end_angle"] == pytest.approx(30.0)
    assert len(result["points"]) == 12
    assert result["points"][0] == (500, 200)


def test_parse_ld06_packet_bad_header_rejected():
    packet = bytearray(_build_ld06_packet(0.0, 30.0, [500] * 12))
    packet[0] = 0x00
    assert pp.parse_ld06_packet(bytes(packet)) is None


# ----------------------------
# publish_speed_command / publish_lidar_steering (file I/O)
# ----------------------------
def test_publish_speed_command_writes_expected_json(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    monkeypatch.setattr(pp, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(pp, "SPEED_STATE_FILE", str(state_dir / "speed_command.json"))

    pp.publish_speed_command(42.5, 1234.0, goal_reached=True)

    with open(state_dir / "speed_command.json") as f:
        data = json.load(f)
    assert data["speed_percent"] == 42.5
    assert data["min_obstacle_dist_mm"] == 1234.0
    assert data["goal_reached"] is True
    assert "timestamp" in data


def test_publish_lidar_steering_writes_expected_json(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    monkeypatch.setattr(pp, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(pp, "LIDAR_STEERING_FILE", str(state_dir / "lidar_steering.json"))

    pp.publish_lidar_steering(12.5)

    with open(state_dir / "lidar_steering.json") as f:
        data = json.load(f)
    assert data["angle_deg"] == 12.5


# ----------------------------
# run_cycle integration smoke tests
# ----------------------------
class _Args:
    def __init__(self, **kw):
        self.pos_candidates_parsed = None
        self.angle_step = 30
        self.inflate = 2
        self.planner = "astar"
        self.goal_tolerance = 200.0
        self.rrt_iter = 500
        self.rrt_step = 100.0
        self.rrt_goal_tolerance = 100.0
        self.map_dir = "."
        for k, v in kw.items():
            setattr(self, k, v)


def test_run_cycle_reports_arrived_when_within_tolerance(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    monkeypatch.setattr(pp, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(pp, "SPEED_STATE_FILE", str(state_dir / "speed_command.json"))
    monkeypatch.setattr(pp, "LIDAR_STEERING_FILE", str(state_dir / "lidar_steering.json"))

    map_points = _make_wall_cloud()
    static_grid = np.zeros((41, 41), dtype=np.uint8)
    origin = (-1000.0, -1000.0)
    resolution = 50.0
    # robot sitting at (0,0,0): in that pose the lidar-frame scan equals the world-frame map
    # points directly, giving ICP a real (non-degenerate) shape to converge on.
    current_scan = map_points.copy()

    x, y, theta, err, path_world, arrived = pp.run_cycle(
        current_scan, map_points, static_grid, origin, resolution,
        goal_xy=(0.0, 0.0), prev_pose=(0.0, 0.0, 0.0), args=_Args(), save_debug_files=False,
    )
    assert arrived is True
    assert path_world is None

    with open(state_dir / "speed_command.json") as f:
        data = json.load(f)
    assert data["speed_percent"] == 0.0
    assert data["goal_reached"] is True


def test_run_cycle_plans_path_toward_distant_goal(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    monkeypatch.setattr(pp, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(pp, "SPEED_STATE_FILE", str(state_dir / "speed_command.json"))
    monkeypatch.setattr(pp, "LIDAR_STEERING_FILE", str(state_dir / "lidar_steering.json"))

    map_points = _make_wall_cloud()
    static_grid = np.zeros((41, 41), dtype=np.uint8)
    origin = (-1000.0, -1000.0)
    resolution = 50.0
    current_scan = map_points.copy()

    x, y, theta, err, path_world, arrived = pp.run_cycle(
        current_scan, map_points, static_grid, origin, resolution,
        goal_xy=(500.0, 500.0), prev_pose=(0.0, 0.0, 0.0), args=_Args(), save_debug_files=False,
    )
    assert arrived is False
    assert path_world is not None
    assert len(path_world) > 0

    with open(state_dir / "lidar_steering.json") as f:
        steering = json.load(f)
    assert -pp.MAX_STEERING_DEG <= steering["angle_deg"] <= pp.MAX_STEERING_DEG
