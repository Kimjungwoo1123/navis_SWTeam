import json

import throttle_control as tc


def _write_cmd(path, speed_percent, min_obstacle_dist_mm, timestamp):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump({
            "speed_percent": speed_percent,
            "min_obstacle_dist_mm": min_obstacle_dist_mm,
            "goal_reached": False,
            "timestamp": timestamp,
        }, f)


def _patch(monkeypatch, tmp_path):
    p = tmp_path / "speed_command.json"
    monkeypatch.setattr(tc, "SPEED_STATE_FILE", str(p))
    return p


def test_read_latest_speed_command_missing_file(tmp_path, monkeypatch):
    _patch(monkeypatch, tmp_path)
    assert tc.read_latest_speed_command() is None


def test_read_latest_speed_command_corrupt(tmp_path, monkeypatch):
    p = _patch(monkeypatch, tmp_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text("not json")
    assert tc.read_latest_speed_command() is None


def test_resolve_target_speed_no_command_is_zero(tmp_path, monkeypatch):
    _patch(monkeypatch, tmp_path)
    assert tc.resolve_target_speed(1000.0) == 0.0


def test_resolve_target_speed_stale_command_is_zero(tmp_path, monkeypatch):
    p = _patch(monkeypatch, tmp_path)
    _write_cmd(p, 50.0, 5000.0, 1000.0)
    now = 1000.0 + tc.STALE_COMMAND_TIMEOUT_S + 1.0
    assert tc.resolve_target_speed(now) == 0.0


def test_resolve_target_speed_fresh_command_passes_through(tmp_path, monkeypatch):
    p = _patch(monkeypatch, tmp_path)
    _write_cmd(p, 35.0, 5000.0, 1000.0)
    assert tc.resolve_target_speed(1000.5) == 35.0


def test_resolve_target_speed_safety_stop_overrides_command(tmp_path, monkeypatch):
    p = _patch(monkeypatch, tmp_path)
    _write_cmd(p, 80.0, tc.SAFETY_STOP_DIST_MM - 1.0, 1000.0)
    assert tc.resolve_target_speed(1000.5) == 0.0


def test_resolve_target_speed_safety_boundary_not_triggered_when_equal(tmp_path, monkeypatch):
    p = _patch(monkeypatch, tmp_path)
    _write_cmd(p, 80.0, tc.SAFETY_STOP_DIST_MM, 1000.0)  # exactly at threshold, not "<"
    assert tc.resolve_target_speed(1000.5) == 80.0


def test_resolve_target_speed_clamps_to_100(tmp_path, monkeypatch):
    p = _patch(monkeypatch, tmp_path)
    _write_cmd(p, 500.0, 5000.0, 1000.0)
    assert tc.resolve_target_speed(1000.5) == 100.0


def test_resolve_target_speed_no_obstacle_info_still_works(tmp_path, monkeypatch):
    p = _patch(monkeypatch, tmp_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w") as f:
        json.dump({"speed_percent": 20.0, "timestamp": 1000.0}, f)  # no min_obstacle_dist_mm key
    assert tc.resolve_target_speed(1000.5) == 20.0


def test_publish_throttle_output_writes_json(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    out_file = state_dir / "throttle_output.json"
    monkeypatch.setattr(tc, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(tc, "THROTTLE_OUTPUT_FILE", str(out_file))

    tc.publish_throttle_output(-25.0)
    with open(out_file) as f:
        data = json.load(f)
    assert data["speed_percent"] == -25.0


def test_accel_ramp_logic_increases_gradually():
    # mirror the ramp logic in main(): should not jump straight to target
    current = 0.0
    target = 50.0
    if target > current:
        current = min(target, current + tc.ACCEL_STEP_PERCENT)
    assert current == tc.ACCEL_STEP_PERCENT
    assert current < target


def test_accel_ramp_logic_decreases_gradually():
    current = 50.0
    target = 0.0
    if target > current:
        current = min(target, current + tc.ACCEL_STEP_PERCENT)
    else:
        current = max(target, current - tc.ACCEL_STEP_PERCENT)
    assert current == 50.0 - tc.ACCEL_STEP_PERCENT
