import json

import kill_switch as ks


def test_publish_estop_writes_active_true(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    estop_file = state_dir / "estop.json"
    monkeypatch.setattr(ks, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(ks, "ESTOP_FILE", str(estop_file))

    ks.publish_estop(True)
    with open(estop_file) as f:
        data = json.load(f)
    assert data["active"] is True
    assert "timestamp" in data


def test_publish_estop_writes_active_false(tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    estop_file = state_dir / "estop.json"
    monkeypatch.setattr(ks, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(ks, "ESTOP_FILE", str(estop_file))

    ks.publish_estop(True)
    ks.publish_estop(False)
    with open(estop_file) as f:
        data = json.load(f)
    assert data["active"] is False


def test_publish_estop_creates_state_dir_if_missing(tmp_path, monkeypatch):
    state_dir = tmp_path / "new_state_dir"
    estop_file = state_dir / "estop.json"
    monkeypatch.setattr(ks, "STATE_DIR", str(state_dir))
    monkeypatch.setattr(ks, "ESTOP_FILE", str(estop_file))
    assert not state_dir.exists()

    ks.publish_estop(False)
    assert state_dir.exists()
    assert estop_file.exists()
