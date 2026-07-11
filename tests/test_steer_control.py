import json
import time

import steer_control as sc


def _write(path, angle_deg, timestamp):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump({"angle_deg": angle_deg, "timestamp": timestamp}, f)


def test_read_angle_file_missing_returns_none(tmp_path):
    assert sc.read_angle_file(str(tmp_path / "nope.json")) is None


def test_read_angle_file_corrupt_returns_none(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{not valid json")
    assert sc.read_angle_file(str(p)) is None


def test_read_angle_file_missing_key_returns_none(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text(json.dumps({"angle_deg": 5.0}))  # no "timestamp"
    assert sc.read_angle_file(str(p)) is None


def test_read_angle_file_valid(tmp_path):
    p = tmp_path / "ok.json"
    _write(p, 12.5, 100.0)
    assert sc.read_angle_file(str(p)) == (12.5, 100.0)


def _patch_files(monkeypatch, tmp_path):
    cam = tmp_path / "camera_steering.json"
    lidar = tmp_path / "lidar_steering.json"
    monkeypatch.setattr(sc, "CAMERA_STEERING_FILE", str(cam))
    monkeypatch.setattr(sc, "LIDAR_STEERING_FILE", str(lidar))
    return cam, lidar


def test_resolve_steering_both_missing_returns_none(tmp_path, monkeypatch):
    _patch_files(monkeypatch, tmp_path)
    angle, source = sc.resolve_steering_angle(1000.0)
    assert angle is None
    assert source == "none(both stale)"


def test_resolve_steering_only_camera_fresh(tmp_path, monkeypatch):
    cam, lidar = _patch_files(monkeypatch, tmp_path)
    _write(cam, 7.0, 1000.0)
    angle, source = sc.resolve_steering_angle(1000.0 + sc.CAMERA_STALE_TIMEOUT_S / 2)
    assert angle == 7.0
    assert source == "camera(lidar stale)"


def test_resolve_steering_only_lidar_fresh(tmp_path, monkeypatch):
    cam, lidar = _patch_files(monkeypatch, tmp_path)
    _write(lidar, -4.0, 1000.0)
    angle, source = sc.resolve_steering_angle(1000.0 + sc.LIDAR_STALE_TIMEOUT_S / 2)
    assert angle == -4.0
    assert source == "lidar(camera stale)"


def test_resolve_steering_camera_stale_lidar_fresh_uses_lidar(tmp_path, monkeypatch):
    cam, lidar = _patch_files(monkeypatch, tmp_path)
    now = 1000.0
    _write(cam, 7.0, now - sc.CAMERA_STALE_TIMEOUT_S - 0.1)  # just stale
    _write(lidar, -4.0, now)
    angle, source = sc.resolve_steering_angle(now)
    assert angle == -4.0
    assert source == "lidar(camera stale)"


def test_resolve_steering_both_fresh_and_agree_uses_lidar(tmp_path, monkeypatch):
    cam, lidar = _patch_files(monkeypatch, tmp_path)
    now = 1000.0
    _write(cam, 10.0, now)
    _write(lidar, 12.0, now)  # diff=2 <= tolerance(5)
    angle, source = sc.resolve_steering_angle(now)
    assert angle == 12.0
    assert source == "lidar(agree)"


def test_resolve_steering_both_fresh_but_disagree_uses_camera(tmp_path, monkeypatch):
    cam, lidar = _patch_files(monkeypatch, tmp_path)
    now = 1000.0
    _write(cam, 10.0, now)
    _write(lidar, 25.0, now)  # diff=15 > tolerance(5)
    angle, source = sc.resolve_steering_angle(now)
    assert angle == 10.0
    assert source == "camera(disagree)"


def test_resolve_steering_agreement_boundary_is_inclusive(tmp_path, monkeypatch):
    cam, lidar = _patch_files(monkeypatch, tmp_path)
    now = 1000.0
    _write(cam, 10.0, now)
    _write(lidar, 15.0, now)  # diff exactly == tolerance(5.0)
    angle, source = sc.resolve_steering_angle(now)
    assert source == "lidar(agree)"


def test_publish_steer_output_writes_json(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    out_file = state_dir / "steer_output.json"
    monkeypatch.setattr(sc, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(sc, "STEER_OUTPUT_FILE", str(out_file))

    sc.publish_steer_output(9.5)
    with open(out_file) as f:
        data = json.load(f)
    assert data["angle_deg"] == 9.5
    assert "timestamp" in data
