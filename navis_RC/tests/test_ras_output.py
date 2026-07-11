import json
import threading
import time

import Ras_output as ro


# ----------------------------
# _clamp / deg_to_can_angle
# ----------------------------
def test_clamp_within_range():
    assert ro._clamp(5, 0, 10) == 5


def test_clamp_below_range():
    assert ro._clamp(-5, 0, 10) == 0


def test_clamp_above_range():
    assert ro._clamp(15, 0, 10) == 10


def test_deg_to_can_angle_center():
    assert ro.deg_to_can_angle(0.0) == ro.ANGLE_CENTER


def test_deg_to_can_angle_full_left_right():
    assert ro.deg_to_can_angle(ro.MAX_STEERING_DEG) == ro.ANGLE_MAX
    assert ro.deg_to_can_angle(-ro.MAX_STEERING_DEG) == ro.ANGLE_MIN


def test_deg_to_can_angle_out_of_range_clamped():
    assert ro.deg_to_can_angle(999.0) == ro.ANGLE_MAX
    assert ro.deg_to_can_angle(-999.0) == ro.ANGLE_MIN


def test_deg_to_can_angle_midpoint():
    half = ro.MAX_STEERING_DEG / 2
    expected = ro.ANGLE_CENTER + half * ((ro.ANGLE_MAX - ro.ANGLE_CENTER) / ro.MAX_STEERING_DEG)
    assert ro.deg_to_can_angle(half) == expected


# ----------------------------
# _read_output_file / read_steer_output / read_throttle_output
# ----------------------------
def test_read_output_file_missing_returns_none(tmp_path):
    assert ro._read_output_file(str(tmp_path / "nope.json"), "angle_deg") is None


def test_read_output_file_stale_returns_none(tmp_path):
    p = tmp_path / "steer.json"
    p.write_text(json.dumps({"angle_deg": 5.0, "timestamp": time.time() - 10}))
    assert ro._read_output_file(str(p), "angle_deg") is None


def test_read_output_file_fresh_returns_value(tmp_path):
    p = tmp_path / "steer.json"
    p.write_text(json.dumps({"angle_deg": 5.0, "timestamp": time.time()}))
    assert ro._read_output_file(str(p), "angle_deg") == 5.0


def test_read_output_file_corrupt_returns_none(tmp_path):
    p = tmp_path / "steer.json"
    p.write_text("{broken")
    assert ro._read_output_file(str(p), "angle_deg") is None


def test_read_steer_output_defaults_to_neutral(tmp_path, monkeypatch):
    monkeypatch.setattr(ro, "STEER_OUTPUT_FILE", str(tmp_path / "nope.json"))
    assert ro.read_steer_output() == 0.0


def test_read_throttle_output_defaults_to_zero(tmp_path, monkeypatch):
    monkeypatch.setattr(ro, "THROTTLE_OUTPUT_FILE", str(tmp_path / "nope.json"))
    assert ro.read_throttle_output() == 0.0


# ----------------------------
# read_estop_active
# ----------------------------
def test_read_estop_active_missing_file_defaults_false(tmp_path, monkeypatch):
    monkeypatch.setattr(ro, "ESTOP_FILE", str(tmp_path / "nope.json"))
    assert ro.read_estop_active() is False


def test_read_estop_active_true(tmp_path, monkeypatch):
    p = tmp_path / "estop.json"
    p.write_text(json.dumps({"active": True, "timestamp": time.time()}))
    monkeypatch.setattr(ro, "ESTOP_FILE", str(p))
    assert ro.read_estop_active() is True


def test_read_estop_active_stale_file_still_true_fail_safe(tmp_path, monkeypatch):
    # by design this does NOT check staleness -- a dead kill_switch process should
    # leave the car stopped, not silently resume
    p = tmp_path / "estop.json"
    p.write_text(json.dumps({"active": True, "timestamp": time.time() - 9999}))
    monkeypatch.setattr(ro, "ESTOP_FILE", str(p))
    assert ro.read_estop_active() is True


def test_read_estop_active_corrupt_file_defaults_false(tmp_path, monkeypatch):
    p = tmp_path / "estop.json"
    p.write_text("{broken")
    monkeypatch.setattr(ro, "ESTOP_FILE", str(p))
    assert ro.read_estop_active() is False


# ----------------------------
# Stm32Link.check_watchdog -- constructed without touching real CAN hardware
# ----------------------------
def _make_link(last_speed=0, last_angle=None):
    link = ro.Stm32Link.__new__(ro.Stm32Link)
    link._last_speed = last_speed
    link._last_angle = last_angle if last_angle is not None else ro.ANGLE_CENTER
    link._status_lock = threading.Lock()
    link._last_status = None
    link._last_status_time = None
    link._mismatch_since = None
    return link


def test_watchdog_no_status_ever_received_is_not_ok():
    link = _make_link()
    ok, reason = link.check_watchdog()
    assert ok is False
    assert "응답" in reason or "통신" in reason


# ----------------------------
# wait_for_first_status -- avoids the "통신 두절 의심" cold-start message on every startup
# ----------------------------
def test_wait_for_first_status_returns_true_once_status_arrives():
    link = _make_link()
    link._last_status = (0, ro.ANGLE_CENTER)
    link._last_status_time = time.time()  # simulate the receive thread already having a status
    assert link.wait_for_first_status(timeout=0.5) is True


def test_wait_for_first_status_times_out_when_nothing_arrives():
    link = _make_link()
    assert link.wait_for_first_status(timeout=0.1) is False


def test_watchdog_status_timed_out_is_not_ok():
    link = _make_link(last_speed=50, last_angle=90)
    link._last_status = (50, 90)
    link._last_status_time = time.time() - ro.STATUS_RECV_TIMEOUT_S - 1.0
    ok, reason = link.check_watchdog()
    assert ok is False


def test_watchdog_matching_status_is_ok():
    link = _make_link(last_speed=50, last_angle=90)
    link._last_status = (50, 90)
    link._last_status_time = time.time()
    ok, reason = link.check_watchdog()
    assert ok is True


def test_watchdog_small_mismatch_within_tolerance_is_ok():
    link = _make_link(last_speed=50, last_angle=90)
    link._last_status = (50 + ro.STATUS_MISMATCH_SPEED_TOLERANCE, 90)
    link._last_status_time = time.time()
    ok, reason = link.check_watchdog()
    assert ok is True


def test_watchdog_large_mismatch_grace_period_still_ok_on_first_check():
    link = _make_link(last_speed=50, last_angle=90)
    link._last_status = (50 + ro.STATUS_MISMATCH_SPEED_TOLERANCE + 50, 90)
    link._last_status_time = time.time()
    ok, reason = link.check_watchdog()
    # first detection of mismatch should not immediately fail (ramp-up grace period)
    assert ok is True
    assert link._mismatch_since is not None


def test_watchdog_large_mismatch_persisting_eventually_fails():
    link = _make_link(last_speed=50, last_angle=90)
    link._last_status = (50 + ro.STATUS_MISMATCH_SPEED_TOLERANCE + 50, 90)
    link._last_status_time = time.time()
    ok, _ = link.check_watchdog()
    assert ok is True  # grace period

    # simulate time passing beyond STATUS_MISMATCH_PERSIST_S without recovery
    link._mismatch_since = time.time() - ro.STATUS_MISMATCH_PERSIST_S - 0.1
    link._last_status_time = time.time()
    ok, reason = link.check_watchdog()
    assert ok is False


def test_watchdog_mismatch_recovering_resets_state():
    link = _make_link(last_speed=50, last_angle=90)
    link._last_status = (150, 90)  # big mismatch
    link._last_status_time = time.time()
    link.check_watchdog()
    assert link._mismatch_since is not None

    # now status catches up to command
    link._last_status = (50, 90)
    link._last_status_time = time.time()
    ok, reason = link.check_watchdog()
    assert ok is True
    assert link._mismatch_since is None
    assert reason == "ok"
