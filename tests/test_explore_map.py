import json
import struct

import numpy as np
import pytest

import explore_and_map as em


class _FakeSerial:
    """buffer 안의 바이트를 read(n)으로 조금씩 흘려주는 가짜 시리얼 포트."""

    def __init__(self, data):
        self.data = data
        self.pos = 0

    def read(self, n):
        chunk = self.data[self.pos:self.pos + n]
        self.pos += len(chunk)
        return chunk


def _build_ld06_packet(start_angle_deg, end_angle_deg, distances):
    header = bytes([0x54, 0x2C])
    speed = struct.pack('<H', 3000)
    start = struct.pack('<H', int(start_angle_deg * 100))
    body = b"".join(struct.pack('<H', d) + bytes([200]) for d in distances)
    end = struct.pack('<H', int(end_angle_deg * 100))
    timestamp = struct.pack('<H', 0)
    crc = bytes([0])
    return header + speed + start + body + end + timestamp + crc


def test_sync_to_ld06_header_finds_header_after_garbage_prefix():
    packet = _build_ld06_packet(0.0, 30.0, [500] * 12)
    ser = _FakeSerial(b"\x01\x02\x03garbage" + packet)
    assert em.sync_to_ld06_header(ser) is True
    rest = ser.read(45)
    assert bytes([0x54, 0x2C]) + rest == packet


def test_read_one_rotation_recovers_when_stream_starts_mid_packet():
    starts = list(range(0, 360, 30))
    rotation_packets = b"".join(_build_ld06_packet(float(s), float(s + 30), [500] * 12) for s in starts)
    one_extra_wrap_packet = _build_ld06_packet(0.0, 30.0, [500] * 12)
    full_stream = rotation_packets + one_extra_wrap_packet
    misaligned_prefix = full_stream[20:47]
    ser = _FakeSerial(misaligned_prefix + full_stream)

    points = em.read_one_rotation(ser, min_points=60, max_wait_s=3.0)
    assert len(points) >= 60


# ----------------------------
# grid <-> world (shared logic with Path_Planning)
# ----------------------------
def test_world_to_grid_grid_to_world_roundtrip():
    origin = (-800.0, -800.0)
    resolution = 50.0
    for row in range(0, 30, 4):
        for col in range(0, 30, 4):
            x, y = em.grid_to_world(row, col, origin, resolution)
            r2, c2 = em.world_to_grid(x, y, origin, resolution)
            assert (r2, c2) == (row, col)


# ----------------------------
# compute_min_obstacle_dist_mm / predict_closing_stop
# ----------------------------
def test_compute_min_obstacle_dist_mm_matches_path_planning_semantics():
    scan = np.array([[30.0, 40.0], [1000.0, 0.0]])
    assert em.compute_min_obstacle_dist_mm(scan) == pytest.approx(50.0)


def test_predict_closing_stop_triggers_on_fast_approach():
    should_stop, speed = em.predict_closing_stop(100.0, 1100.0, 1.0)
    assert should_stop is True
    assert speed == pytest.approx(1000.0)


def test_predict_closing_stop_no_trigger_when_receding():
    should_stop, speed = em.predict_closing_stop(700.0, 600.0, 1.0)
    assert should_stop is False


# ----------------------------
# init_explore_grid
# ----------------------------
def test_init_explore_grid_all_unknown_and_centered_on_origin():
    grid, origin, resolution = em.init_explore_grid()
    assert (grid == -1).all()
    assert resolution == em.GRID_RESOLUTION_MM
    # world (0,0) (robot start) must map inside the grid
    row, col = em.world_to_grid(0.0, 0.0, origin, resolution)
    h, w = grid.shape
    assert 0 <= row < h and 0 <= col < w


# ----------------------------
# raytrace_update
# ----------------------------
def test_raytrace_update_marks_free_along_ray_and_occupied_at_endpoint():
    grid, origin, resolution = em.init_explore_grid()
    robot_xy = (0.0, 0.0)
    scan_world = np.array([[500.0, 0.0]])
    em.raytrace_update(grid, origin, resolution, robot_xy, scan_world)

    mid_r, mid_c = em.world_to_grid(250.0, 0.0, origin, resolution)
    assert grid[mid_r, mid_c] == 0  # free along the ray

    end_r, end_c = em.world_to_grid(500.0, 0.0, origin, resolution)
    assert grid[end_r, end_c] == 1  # obstacle at the hit point


def test_raytrace_update_does_not_downgrade_confirmed_occupied():
    grid, origin, resolution = em.init_explore_grid()
    # mark a cell OCCUPIED directly ahead of a ray that passes straight through it
    occ_r, occ_c = em.world_to_grid(250.0, 0.0, origin, resolution)
    grid[occ_r, occ_c] = 1

    scan_world = np.array([[500.0, 0.0]])
    em.raytrace_update(grid, origin, resolution, (0.0, 0.0), scan_world)

    assert grid[occ_r, occ_c] == 1  # must remain OCCUPIED, not overwritten back to FREE


# ----------------------------
# to_binary_obstacle_grid / build_planning_grid
# ----------------------------
def test_to_binary_obstacle_grid_treats_unknown_as_obstacle():
    grid = np.array([[-1, 0, 1]], dtype=np.int8)
    binary = em.to_binary_obstacle_grid(grid)
    assert list(binary[0]) == [1, 0, 1]


def test_build_planning_grid_does_not_inflate_unknown():
    grid = np.full((15, 15), -1, dtype=np.int8)
    grid[7, 7] = 1  # a single confirmed obstacle surrounded by unknown
    planning = em.build_planning_grid(grid, inflate_cells=2)
    # everything should be blocked (unknown treated as blocked) except possibly nowhere new opened up
    assert planning[7, 7] == 1
    # a cell far from the obstacle should still just be "unknown blocked", not something new
    assert planning[0, 0] == 1  # unknown -> blocked regardless of inflation


def test_build_planning_grid_inflates_only_confirmed_obstacles():
    grid = np.zeros((15, 15), dtype=np.int8)  # all FREE
    grid[7, 7] = 1
    planning = em.build_planning_grid(grid, inflate_cells=2)
    # neighbor cells around the obstacle should now be blocked due to inflation
    assert planning[7, 8] == 1
    assert planning[7, 7] == 1
    # a cell far away should remain free (0) since only FREE was in the grid besides the one obstacle
    assert planning[0, 0] == 0


# ----------------------------
# is_boxed_in
# ----------------------------
def test_is_boxed_in_true_when_all_four_sides_occupied():
    grid, origin, resolution = em.init_explore_grid()
    check_dist = em.BOXED_IN_CHECK_DIST_MM
    for dx, dy in [(check_dist, 0), (-check_dist, 0), (0, check_dist), (0, -check_dist)]:
        r, c = em.world_to_grid(dx, dy, origin, resolution)
        grid[r, c] = 1
    assert em.is_boxed_in(grid, origin, resolution, 0.0, 0.0, 0.0) is True


def test_is_boxed_in_false_when_one_side_open():
    grid, origin, resolution = em.init_explore_grid()
    check_dist = em.BOXED_IN_CHECK_DIST_MM
    # only mark 3 of 4 sides occupied
    for dx, dy in [(check_dist, 0), (-check_dist, 0), (0, check_dist)]:
        r, c = em.world_to_grid(dx, dy, origin, resolution)
        grid[r, c] = 1
    assert em.is_boxed_in(grid, origin, resolution, 0.0, 0.0, 0.0) is False


def test_is_boxed_in_false_when_sides_only_unknown():
    grid, origin, resolution = em.init_explore_grid()  # all UNKNOWN by default
    assert em.is_boxed_in(grid, origin, resolution, 0.0, 0.0, 0.0) is False


# ----------------------------
# find_frontier_mask / cluster_frontiers
# ----------------------------
def test_find_frontier_mask_identifies_free_next_to_unknown():
    grid = np.full((10, 10), -1, dtype=np.int8)
    grid[5, 5] = 0  # single free cell surrounded by unknown
    frontier = em.find_frontier_mask(grid)
    assert frontier[5, 5] == True  # noqa: E712


def test_find_frontier_mask_excludes_free_surrounded_by_free():
    grid = np.zeros((10, 10), dtype=np.int8)  # all free, no unknown anywhere
    frontier = em.find_frontier_mask(grid)
    assert not frontier.any()


def test_cluster_frontiers_filters_small_clusters():
    grid = np.full((10, 10), -1, dtype=np.int8)
    grid[5, 5] = 0  # cluster of size 1 -- below FRONTIER_MIN_CLUSTER_CELLS default (3)
    frontier = em.find_frontier_mask(grid)
    reps = em.cluster_frontiers(frontier)
    assert reps == []


def test_cluster_frontiers_returns_member_of_actual_cluster():
    grid = np.full((10, 10), -1, dtype=np.int8)
    grid[5, 4:7] = 0  # a row of 3 free cells next to unknown, cluster size >= 3
    frontier = em.find_frontier_mask(grid)
    reps = em.cluster_frontiers(frontier, min_cluster_cells=3)
    assert len(reps) == 1
    rep = reps[0]
    assert frontier[rep[0], rep[1]] == True  # noqa: E712 -- representative must be an actual frontier cell


# ----------------------------
# find_nearby_passable_cell
# ----------------------------
def test_find_nearby_passable_cell_returns_self_if_free():
    grid = np.zeros((10, 10), dtype=np.uint8)
    assert em.find_nearby_passable_cell(grid, 5, 5) == (5, 5)


def test_find_nearby_passable_cell_finds_nearest_free_neighbor():
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[5, 5] = 1  # blocked
    result = em.find_nearby_passable_cell(grid, 5, 5, max_radius=2)
    assert result is not None
    assert grid[result[0], result[1]] == 0


def test_find_nearby_passable_cell_returns_none_if_fully_blocked():
    grid = np.ones((10, 10), dtype=np.uint8)
    assert em.find_nearby_passable_cell(grid, 5, 5, max_radius=2) is None


# ----------------------------
# choose_frontier_goal
# ----------------------------
def test_choose_frontier_goal_no_frontier_when_fully_explored():
    grid = np.zeros((10, 10), dtype=np.int8)  # fully free, no unknown -> no frontier
    inflated = em.build_planning_grid(grid)
    goal, reason = em.choose_frontier_goal(grid, (0.0, 0.0), (-250.0, -250.0), 50.0, inflated)
    assert goal is None
    assert reason == "no_frontier"


def test_choose_frontier_goal_reaches_open_frontier():
    grid = np.full((20, 20), -1, dtype=np.int8)
    grid[8:12, 8:12] = 0  # small explored room around the center, rest unknown
    origin = (-500.0, -500.0)
    resolution = 50.0
    inflated = em.build_planning_grid(grid)
    robot_xy = em.grid_to_world(10, 10, origin, resolution)
    goal, reason = em.choose_frontier_goal(grid, robot_xy, origin, resolution, inflated)
    assert reason == "ok"
    assert goal is not None


# ----------------------------
# astar (own copy, includes bounds + blocked start/goal checks)
# ----------------------------
def test_astar_out_of_bounds_returns_none():
    grid = np.zeros((5, 5), dtype=np.uint8)
    assert em.astar(grid, (-1, 0), (4, 4)) is None
    assert em.astar(grid, (0, 0), (5, 5)) is None


def test_astar_blocked_start_returns_none():
    grid = np.zeros((5, 5), dtype=np.uint8)
    grid[0, 0] = 1
    assert em.astar(grid, (0, 0), (4, 4)) is None


def test_astar_finds_path():
    grid = np.zeros((10, 10), dtype=np.uint8)
    path = em.astar(grid, (0, 0), (9, 9))
    assert path is not None and path[0] == (0, 0) and path[-1] == (9, 9)


# ----------------------------
# pure_pursuit_steering / curvature_speed_factor (same semantics as Path_Planning)
# ----------------------------
def test_pure_pursuit_steering_straight_is_zero():
    path = [(0.0, 0.0), (500.0, 0.0)]
    assert em.pure_pursuit_steering(path, 0.0, 0.0, 0.0) == pytest.approx(0.0, abs=1e-6)


def test_curvature_speed_factor_uses_explore_max_steering_by_default():
    # at EXPLORE_MAX_STEERING_DEG, factor should hit the floor
    assert em.curvature_speed_factor(em.EXPLORE_MAX_STEERING_DEG) == pytest.approx(em.CURVE_SPEED_MIN_FACTOR)


# ----------------------------
# apply_min_moving_floor -- hardware deadzone floor (measured: 50=no motion, 60=motion)
# ----------------------------
def test_apply_min_moving_floor_zero_stays_zero():
    assert em.apply_min_moving_floor(0.0) == 0.0


def test_apply_min_moving_floor_negative_treated_as_stop():
    assert em.apply_min_moving_floor(-5.0) == 0.0


def test_apply_min_moving_floor_raises_below_deadzone_value():
    # CRUISE_SPEED_PERCENT_EXPLORE (~33%) alone is below the real deadzone
    assert em.apply_min_moving_floor(em.CRUISE_SPEED_PERCENT_EXPLORE) == em.MIN_MOVING_SPEED_PERCENT


def test_apply_min_moving_floor_leaves_value_above_floor_untouched():
    assert em.apply_min_moving_floor(80.0) == 80.0


def test_apply_min_moving_floor_clamps_to_full_speed():
    assert em.apply_min_moving_floor(150.0) == em.FULL_SPEED_PERCENT


def test_apply_min_moving_floor_curvature_reduced_speed_never_drops_below_floor():
    # worst case: cruise speed at its lowest legal value, sharpest curve (min factor)
    reduced = em.CRUISE_SPEED_PERCENT_EXPLORE * em.CURVE_SPEED_MIN_FACTOR
    assert reduced < em.MIN_MOVING_SPEED_PERCENT  # sanity: this is exactly the bug scenario
    assert em.apply_min_moving_floor(reduced) == em.MIN_MOVING_SPEED_PERCENT


# ----------------------------
# ICP / localize_with_recovery
# ----------------------------
def _make_wall_cloud(n=400):
    xs = np.linspace(-1000, 1000, n // 2)
    wall1 = np.stack([xs, np.full_like(xs, 1000.0)], axis=1)
    ys = np.linspace(-1000, 1000, n // 2)
    wall2 = np.stack([np.full_like(ys, 1000.0), ys], axis=1)
    return np.vstack([wall1, wall2])


def test_icp_recovers_known_transform():
    target = _make_wall_cloud()
    true_dx, true_dy, true_theta = -60.0, 90.0, -8.0
    th = np.deg2rad(true_theta)
    Rt = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    t = np.array([true_dx, true_dy])
    source = (target - t) @ Rt

    _, R_est, t_est, err = em.icp(source, target, init_guess=(0.0, 0.0, 0.0))
    est_theta = np.degrees(np.arctan2(R_est[1, 0], R_est[0, 0]))

    assert err < 20.0
    assert est_theta == pytest.approx(true_theta, abs=2.0)


def test_localize_with_recovery_trustworthy_on_good_match():
    target = _make_wall_cloud()
    prev_pose = (0.0, 0.0, 0.0)
    current_scan = target.copy()  # scan already in map frame roughly at prev_pose

    x, y, theta, err, mode, consecutive_lost, trustworthy = em.localize_with_recovery(
        current_scan, target, prev_pose, consecutive_lost=0)
    assert bool(trustworthy) is True  # err comparison yields numpy.bool_, not the `True` singleton
    assert mode in ("narrow", "wide", "global")
    assert consecutive_lost == 0


def test_localize_with_recovery_escalates_to_global_after_repeated_failures():
    target = _make_wall_cloud()
    # a scan that looks nothing like the map/target -> should fail to match near prev_pose
    garbage_scan = np.random.default_rng(1).uniform(-50, 50, size=(50, 2))
    prev_pose = (5000.0, 5000.0, 0.0)  # far away from the map entirely

    consecutive_lost = em.GLOBAL_RELOCALIZE_AFTER_CONSECUTIVE_LOST - 1
    x, y, theta, err, mode, consecutive_lost, trustworthy = em.localize_with_recovery(
        garbage_scan, target, prev_pose, consecutive_lost=consecutive_lost)
    assert mode == "global"


# ----------------------------
# publish_speed_command / publish_lidar_steering / export_map (file I/O)
# ----------------------------
def test_publish_speed_command_writes_json(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    monkeypatch.setattr(em, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(em, "SPEED_STATE_FILE", str(state_dir / "speed_command.json"))
    em.publish_speed_command(10.0, 999.0, goal_reached=False)
    with open(state_dir / "speed_command.json") as f:
        data = json.load(f)
    assert data["speed_percent"] == 10.0
    assert data["min_obstacle_dist_mm"] == 999.0


def test_export_map_writes_expected_files(tmp_path):
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 2] = 1
    out_dir = tmp_path / "map_output"
    em.export_map(str(out_dir), np.array([[0.0, 0.0], [10.0, 10.0]]), grid, (-100.0, -100.0), 50.0)

    assert (out_dir / "map_points.csv").exists()
    assert (out_dir / "map_occupancy_grid.npy").exists()
    assert (out_dir / "map_grid_meta.csv").exists()
    assert (out_dir / "map_result.png").exists()

    exported_grid = np.load(out_dir / "map_occupancy_grid.npy")
    assert exported_grid[2, 2] == 1  # confirmed obstacle preserved


# ----------------------------
# run_exploration / return_to_origin integration smoke tests
# ----------------------------
def test_run_exploration_terminates_and_returns_map(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    monkeypatch.setattr(em, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(em, "SPEED_STATE_FILE", str(state_dir / "speed_command.json"))
    monkeypatch.setattr(em, "LIDAR_STEERING_FILE", str(state_dir / "lidar_steering.json"))

    # a small square "room": walls close enough that the robot boxes itself in quickly
    def fake_scan():
        angles = np.linspace(0, 359, 180)
        dists = np.full(180, 300.0)  # walls 300mm away on all sides
        return list(zip(angles, dists))

    map_points, grid, origin, resolution, pose = em.run_exploration(fake_scan, max_cycles=5)
    assert grid.shape[0] > 0
    assert isinstance(pose, tuple) and len(pose) == 3
