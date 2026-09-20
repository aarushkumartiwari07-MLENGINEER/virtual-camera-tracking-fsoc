# Frontend & API Integration Architecture: SIH26169

**Smart India Hackathon (SIH) 2026**  
**Problem Statement ID**: SIH26169 — Development of an AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals  
**Organization**: Department of Space / ISRO  
**Team**: Greedy Minds  
**Document**: Developer Frontend & FastAPI Integration Reference  

---

## 1. Architectural Overview & Boundary Separation

The system follows a strict 3-tier decoupled architecture designed for high maintainability, independent testability, and clean replacement capability:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        DEVELOPER FRONTEND (UI)                         │
│   Vanilla ES6 Modules + Native Canvas 2D + Resilient WebSocket Client  │
│   (Zero framework lock-in, zero build step, 60 FPS HUD/Sensor render)  │
└──────────────────────────────────┬─────────────────────────────────────┘
                                   │ HTTP (REST) / WS (Binary/JSON)
                                   ▼
┌────────────────────────────────────────────────────────────────────────┐
│                    FASTAPI SERVICE LAYER (ADAPTER)                     │
│   - backend/api/server.py             - backend/api/schemas.py         │
│   - backend/api/routes/               - backend/api/websocket_manager.py│
│   - backend/api/services/simulation_service.py                         │
│   - backend/api/services/benchmark_service.py                          │
│                                                                        │
│   * ZERO CV, Kalman, or Controller math in the API layer.              │
└──────────────────────────────────┬─────────────────────────────────────┘
                                   │ Pure Python Dataclass & Frame API
                                   ▼
┌────────────────────────────────────────────────────────────────────────┐
│                          CORE BACKEND SYSTEM                           │
│   - SimulationEngine (backend/simulation/)                             │
│   - VisionPipeline   (backend/vision/)                                 │
│   - BenchmarkRunner  (backend/evaluation/)                             │
│   - TrackingEvaluator (backend/evaluation/)                            │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 2. FastAPI REST Endpoints Contract

All REST endpoints operate under the `/api` prefix:

### 2.1 Simulation Lifecycle (`/api/simulation`)
| Method | Endpoint | Description | Request Body | Response Body |
| :--- | :--- | :--- | :--- | :--- |
| `GET` | `/api/simulation/status` | Current running state, frame index, FPS, simulation time | None | `SimulationStatusResponse` |
| `POST` | `/api/simulation/start` | Starts the 30 Hz asynchronous background simulation loop | None | `CommandResponse` |
| `POST` | `/api/simulation/pause` | Pauses stepping while keeping current state intact | None | `CommandResponse` |
| `POST` | `/api/simulation/resume` | Resumes stepping from paused state | None | `CommandResponse` |
| `POST` | `/api/simulation/stop` | Stops loop and resets simulation to t=0 | None | `CommandResponse` |
| `POST` | `/api/simulation/reset` | Resets state and reinitializes pipeline | None | `CommandResponse` |
| `POST` | `/api/simulation/step` | Single-frame manual tick (useful for debug inspection) | None | `TelemetryPacket` |

### 2.2 System Configuration (`/api/config`)
| Method | Endpoint | Description | Request Body | Response Body |
| :--- | :--- | :--- | :--- | :--- |
| `GET` | `/api/config` | Returns current SimulationConfig & VisionConfig | None | `FullConfigResponse` |
| `PUT` | `/api/config` | Updates camera, beacon, detector, or Kalman parameters | `ConfigUpdateRequest` | `FullConfigResponse` |
| `POST` | `/api/config/reset` | Restores default nominal configuration | None | `FullConfigResponse` |

### 2.3 Tracking & Evaluator Metrics (`/api/tracking` & `/api/metrics`)
| Method | Endpoint | Description | Request Body | Response Body |
| :--- | :--- | :--- | :--- | :--- |
| `GET` | `/api/tracking/state` | Returns latest TrackerOutput state | None | `TrackingStateResponse` |
| `GET` | `/api/metrics` | Returns cumulative TrackingMetrics (RMSE, Loss %, Latency) | None | `TrackingMetricsResponse` |
| `GET` | `/api/metrics/history` | Returns sliding window time-series error data (up to N frames) | Query `limit=100` | `MetricsHistoryResponse` |
| `POST` | `/api/metrics/reset` | Clears accumulated evaluation metrics | None | `CommandResponse` |

### 2.4 Benchmarks (`/api/benchmarks`)
| Method | Endpoint | Description | Request Body | Response Body |
| :--- | :--- | :--- | :--- | :--- |
| `GET` | `/api/benchmarks/scenarios` | Lists 14 preconfigured official SIH benchmark scenarios | None | `BenchmarkScenariosResponse` |
| `POST` | `/api/benchmarks/run` | Triggers background execution of benchmark suite | `BenchmarkRunRequest` | `CommandResponse` |
| `GET` | `/api/benchmarks/latest` | Returns latest benchmark execution status and results table | None | `BenchmarkStatusResponse` |

---

## 3. Real-Time WebSocket Telemetry Protocol (`/ws`)

The real-time streaming channel operates over standard WebSockets (`ws://127.0.0.1:8000/ws`).

### 3.1 Envelope Schema
All WebSocket messages are structured JSON envelopes:
```json
{
  "type": "telemetry" | "event" | "state_change" | "benchmark_progress" | "benchmark_complete" | "connection_ack" | "pong",
  "data": { ... },
  "timestamp": 1789910400.123
}
```

### 3.2 Telemetry Packet Payload (`type: "telemetry"`)
Broadcast at 30 Hz when simulation is running:
```json
{
  "type": "telemetry",
  "data": {
    "frame_index": 142,
    "sim_time": 4.733,
    "state": "RUNNING",
    "world_bounds": {"width": 2000, "height": 2000},
    "ground_truth": {
      "beacon_world_pos": [1085.4, 942.1],
      "beacon_world_vel": [15.2, -8.1],
      "camera_world_pos": [1000.0, 1000.0],
      "camera_pan_deg": 0.0,
      "camera_tilt_deg": 0.0,
      "is_visible": true,
      "target_image_pos": [405.4, 182.1]
    },
    "tracking": {
      "state": "TRACKING",
      "is_detected": true,
      "raw_centroid": [405.3, 182.2],
      "filtered_centroid": [405.1, 182.0],
      "filtered_velocity": [15.0, -8.0],
      "raw_error_px": 0.14,
      "filtered_error_px": 0.32,
      "missed_frames": 0,
      "candidate_count": 1,
      "selected_score": 0.985
    },
    "metrics": {
      "fps": 30.1,
      "processing_time_ms": 0.62,
      "mean_raw_error_px": 0.12,
      "raw_rmse_px": 0.18,
      "filtered_rmse_px": 0.28,
      "target_loss_percentage": 0.0,
      "acquisition_latency_s": 0.033
    },
    "frame_jpeg_b64": "/9j/4AAQSkZJRgABAQAAAQABAAD/..."
  },
  "timestamp": 1789910400.123
}
```

---

## 4. Frontend Architecture & Visualizers

The frontend is implemented in clean, modern Vanilla HTML5/ES6/CSS with zero external framework dependencies.

### Directory Structure:
```
frontend/
├── index.html                  # Application shell & semantic grid layout
├── css/
│   └── style.css               # Aerospace dark-mode HUD styling & design tokens
└── js/
    ├── api.js                  # Typed REST API client
    ├── websocket.js            # Auto-reconnecting WebSocket client
    ├── renderers/
    │   ├── world_renderer.js   # 2000x2000 world coordinate grid & FOV frustum
    │   ├── sensor_renderer.js  # 640x480 sensor canvas with reticles & error vectors
    │   └── charts.js           # Real-time Error vs Time & FSM ribbon chart
    └── app.js                  # Master application controller & state coordinator
```

### Key Visualizers:
1. **World Frustum Canvas (`world_renderer.js`)**:
   - Visualizes the entire 2000×2000 virtual space.
   - Draws coordinate grid with 200-unit spacing.
   - Dynamic FOV footprint polygon reflecting camera position and pan/tilt pointing angles.
   - Multi-point beacon trajectory trail with distinct color coding for in-FOV vs out-of-FOV states.
2. **Camera Sensor Canvas (`sensor_renderer.js`)**:
   - Renders the live 8-bit monochrome sensor image.
   - Optical bore-sight crosshair at $(W/2, H/2)$.
   - Ground Truth reticle (Green `+`).
   - Raw Classical CV Centroid (Cyan `✕`).
   - Kalman Filtered Centroid (Amber `◯`) with dynamic velocity projection vector.
   - Direct Error Vector connecting Kalman centroid to bore-sight center.
3. **Telemetry & Real-Time Performance HUD**:
   - Live FPS and sub-millisecond CV processing time.
   - Instantaneous and cumulative RMSE.
   - Acquisition latency and Target Loss %.
4. **State Machine Ribbon & Error vs. Time Chart (`charts.js`)**:
   - Real-time sliding window (200 frames) graphing Raw Error vs Kalman Error against ground truth.
   - FSM state timeline ribbon displaying operational state transitions (`SEARCHING`, `ACQUIRED`, `TRACKING`, `LOST`, `REACQUIRED`).
5. **Phase 3 Gimbal Control Tab**:
   - Cleanly exposes PID parameter placeholders, pan/tilt limits, and angular error readouts, clearly labeled as **"PENDING PHASE 3 IMPLEMENTATION"**.

---

## 5. Development & Running Instructions

### Starting the Server:
Run the FastAPI development server from the repository root:
```bash
python -m uvicorn backend.api.server:app --host 127.0.0.1 --port 8000 --reload
```

### Accessing the Developer Interface:
Open your browser to:
```
http://127.0.0.1:8000/
```
Interactive Swagger API documentation is available at:
```
http://127.0.0.1:8000/docs
```

### Running the Test Suite:
Run all backend unit, integration, and API tests:
```bash
pytest -v
```
