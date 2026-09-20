"""
Comprehensive REST API and WebSocket Integration Tests for FastAPI Service Layer.
Verifies status endpoints, simulation lifecycle commands, configuration CRUD,
metrics reporting, and benchmark execution using FastAPI TestClient.
"""

import pytest
from fastapi.testclient import TestClient
from backend.api.server import app


@pytest.fixture(scope="module")
def client():
    """Create a synchronous FastAPI test client."""
    with TestClient(app) as test_client:
        yield test_client


def test_api_simulation_status(client):
    """Verify GET /api/simulation/status returns valid state."""
    response = client.get("/api/simulation/status")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data
    assert "frame_index" in data
    assert "fps" in data
    assert "target_fps" in data
    assert "motion_type" in data
    assert "tracking_state" in data


def test_api_simulation_lifecycle_flow(client):
    """Verify start -> pause -> resume -> step -> reset -> stop command sequence."""
    # 1. Start
    res = client.post("/api/simulation/start")
    assert res.status_code == 200
    assert res.json()["success"] is True
    assert res.json()["status"] == "RUNNING"

    # 2. Pause
    res = client.post("/api/simulation/pause")
    assert res.status_code == 200
    assert res.json()["status"] == "PAUSED"

    # 3. Step
    res = client.post("/api/simulation/step")
    assert res.status_code == 200
    assert res.json()["success"] is True
    assert "telemetry" in res.json()
    assert res.json()["telemetry"]["frame_index"] >= 1

    # 4. Resume
    res = client.post("/api/simulation/resume")
    assert res.status_code == 200
    assert res.json()["status"] == "RUNNING"

    # 5. Stop
    res = client.post("/api/simulation/stop")
    assert res.status_code == 200
    assert res.json()["status"] == "STOPPED"

    # 6. Reset with figure_eight motion
    res = client.post("/api/simulation/reset?motion=figure_eight")
    assert res.status_code == 200
    assert res.json()["success"] is True


def test_api_configuration_crud(client):
    """Verify GET /api/config, PUT /api/config, and POST /api/config/reset."""
    # 1. Get current config
    res = client.get("/api/config")
    assert res.status_code == 200
    cfg = res.json()
    assert "simulation" in cfg
    assert "detector" in cfg
    assert "kalman" in cfg
    assert "state_machine" in cfg

    # 2. Modify config
    cfg["simulation"]["fps"] = 45.0
    cfg["simulation"]["beacon"]["size"] = 15.0
    cfg["kalman"]["process_noise_std"] = 100.0

    put_res = client.put("/api/config", json=cfg)
    assert put_res.status_code == 200
    assert put_res.json()["success"] is True

    # Verify changes applied
    get_res = client.get("/api/config")
    assert get_res.json()["simulation"]["fps"] == 45.0
    assert get_res.json()["simulation"]["beacon"]["size"] == 15.0
    assert get_res.json()["kalman"]["process_noise_std"] == 100.0

    # 3. Reset to default
    reset_res = client.post("/api/config/reset")
    assert reset_res.status_code == 200
    assert reset_res.json()["success"] is True

    # Verify reset to 30.0 fps
    final_res = client.get("/api/config")
    assert final_res.json()["simulation"]["fps"] == 30.0


def test_api_tracking_and_metrics(client):
    """Verify /api/tracking/state, /api/metrics, /api/metrics/history, and reset."""
    # Step a few frames to generate metrics
    for _ in range(5):
        client.post("/api/simulation/step")

    # 1. Tracking State
    state_res = client.get("/api/tracking/state")
    assert state_res.status_code == 200
    assert "status" in state_res.json()
    assert "latest_telemetry" in state_res.json()

    # 2. Metrics Summary
    metrics_res = client.get("/api/metrics")
    assert metrics_res.status_code == 200
    m = metrics_res.json()
    assert "total_frames" in m
    assert "detection_rate_pct" in m
    assert "rmse_raw_error_px" in m
    assert "rmse_filtered_error_px" in m
    assert m["total_frames"] >= 5

    # 3. Metrics History
    hist_res = client.get("/api/metrics/history")
    assert hist_res.status_code == 200
    h = hist_res.json()
    assert "error_history" in h
    assert "state_history" in h
    assert len(h["error_history"]) >= 5

    # 4. Metrics Reset
    reset_res = client.post("/api/metrics/reset")
    assert reset_res.status_code == 200
    assert reset_res.json()["success"] is True

    # Verify reset
    new_metrics = client.get("/api/metrics").json()
    assert new_metrics["total_frames"] == 0


def test_api_benchmarks(client):
    """Verify benchmark scenario listing and single-scenario execution."""
    # 1. Get Scenarios
    scenarios_res = client.get("/api/benchmarks/scenarios")
    assert scenarios_res.status_code == 200
    scenarios = scenarios_res.json()
    assert len(scenarios) >= 10
    assert any(s["id"] == "A1_static_clean" for s in scenarios)

    # 2. Run Single Fast Benchmark Scenario
    req_payload = {
        "scenario_id": "B1_linear_clean",
        "motion_type": "straight_line",
        "shape": "gaussian",
        "noise_std_px": 0.0,
        "steps": 30,
        "fps": 30.0,
    }
    run_res = client.post("/api/benchmarks/run", json=req_payload)
    assert run_res.status_code == 200
    res_data = run_res.json()
    assert res_data["success"] is True
    assert len(res_data["results"]) == 1
    r = res_data["results"][0]
    assert r["raw_rmse_px"] >= 0.0
    assert r["filtered_rmse_px"] >= 0.0

    # 3. Get Latest Benchmark Results
    latest_res = client.get("/api/benchmarks/latest")
    assert latest_res.status_code == 200
    assert len(latest_res.json()) >= 1


def test_websocket_telemetry_handshake(client):
    """Verify WebSocket endpoint /ws establishes connection and sends initial ack."""
    with client.websocket_connect("/ws") as websocket:
        # First message should be connection_ack
        ack = websocket.receive_json()
        assert ack["type"] == "connection_ack"
        assert "status" in ack

        # Next message is the initial telemetry frame
        telemetry_msg = websocket.receive_json()
        assert telemetry_msg["type"] == "telemetry"

        # Send ping
        websocket.send_json({"command": "ping"})
        pong = websocket.receive_json()
        assert pong["type"] == "pong"
