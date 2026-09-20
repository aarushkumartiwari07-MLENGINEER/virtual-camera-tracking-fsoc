# AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals (Backend)

**Smart India Hackathon (SIH) 2026** — Problem Statement ID: **SIH26169**  
**Organization**: Department of Space / ISRO  
**Category**: Software  
**Theme**: Smart Automation / Space Technology  
**Team**: Greedy Minds  

---

## 1. Overview

This repository contains the pure-Python, frontend-agnostic simulation and tracking backend for Free Space Optical Communication (FSOC) coarse terminal alignment. 

Free space optical communication links require establishing line-of-sight pointing before fine tracking (using Fast Steering Mirrors or quadrant photodetectors) can operate. This project provides a 100% software-based simulation environment that models:
- A wide-area virtual tracking space ($2000 \times 2000$ px).
- Dynamic optical beacons with configurable trajectories (Straight-line, Circular, Figure-of-8, Random walk).
- A virtual PTZ receiver camera ($640 \times 480$, $4.0^\circ \times 3.0^\circ$ FOV) with physical pan/tilt kinematics.
- Mathematically consistent angular and perspective projection.
- High-fidelity monochrome sensor frame rendering.
- Authoritative ground-truth state generation for tracking metrics and benchmarking.

---

## 2. Project Architecture

```
26169backend/
├── backend/
│   ├── __init__.py
│   ├── engine.py                   # Master frontend-agnostic SimulationEngine API
│   ├── api/                        # FastAPI Integration & Service Layer
│   │   ├── __init__.py
│   │   ├── server.py               # FastAPI application, CORS, static mounting
│   │   ├── schemas.py              # Pydantic models for REST & WebSocket payloads
│   │   ├── websocket_manager.py    # Real-time WebSocket connection manager (/ws)
│   │   ├── routes/                 # Thin REST route handlers
│   │   │   ├── simulation_routes.py
│   │   │   ├── config_routes.py
│   │   │   ├── tracking_routes.py
│   │   │   └── benchmark_routes.py
│   │   └── services/               # Background task & simulation loop runners
│   │       ├── simulation_service.py
│   │       └── benchmark_service.py
│   ├── config/
│   │   ├── __init__.py
│   │   ├── simulation_config.py    # Dataclasses for World, Beacon, Camera, Simulation
│   │   └── vision_config.py        # Config dataclasses for Detector, Centroid, Kalman, FSM
│   ├── models/
│   │   ├── __init__.py
│   │   ├── state.py                # Authoritative GroundTruthState definition
│   │   ├── detection.py            # CandidateBlob & DetectionResult dataclasses
│   │   └── tracking_state.py       # TrackingState enum & TrackerOutput dataclass
│   ├── simulation/
│   │   ├── __init__.py
│   │   ├── world.py                # 2000x2000 virtual world & boundary constraints
│   │   ├── beacon.py               # Optical beacon entity
│   │   ├── motion.py               # Straight-line, Circular, Figure-8, Random motion models
│   │   ├── camera.py               # Virtual camera with PTZ gimbal kinematics & rate limits
│   │   ├── projection.py           # Mathematical World -> Angle -> Pixel projection
│   │   └── renderer.py             # 8-bit monochrome sensor frame rasterizer
│   ├── vision/
│   │   ├── __init__.py
│   │   ├── centroid.py             # Intensity-weighted spatial moments & subpixel estimation
│   │   ├── detector.py             # Classical CV blob detector & deterministic scoring
│   │   ├── kalman.py               # 2D/4D constant-velocity Kalman filter with coasting
│   │   ├── state_machine.py        # 5-State Tracking FSM (SEARCHING, ACQUIRED, TRACKING, etc.)
│   │   └── pipeline.py             # Unified VisionPipeline (Pure frame input, 0% ground truth)
│   └── evaluation/
│       ├── __init__.py
│       ├── evaluator.py            # Independent TrackingEvaluator comparing Tracker to Ground Truth
│       └── benchmark.py            # BenchmarkRunner executing parameterized test matrices
├── frontend/                       # Developer Frontend (Aerospace Mission Control UI)
│   ├── index.html                  # Responsive multi-panel HUD layout
│   ├── css/
│   │   └── style.css               # Clean dark-mode engineering theme
│   └── js/
│       ├── api.js                  # REST API client
│       ├── websocket.js            # Auto-reconnecting WebSocket client
│       ├── renderers/              # Native Canvas 2D renderers (World, Sensor, Charts)
│       └── app.js                  # Master UI controller
├── visualization/
│   ├── __init__.py
│   └── viewer.py                   # Real-time desktop visualizer with GT, Raw, Kalman overlays & HUD
├── tests/
│   ├── test_api.py                 # FastAPI REST and WebSocket integration tests
│   ├── test_kalman_audit.py        # Post-audit Kalman accuracy tests
│   ├── test_projection.py          # Projection math, FOV clipping, invertibility tests
│   ├── test_motion.py              # Motion trajectory & boundary behavior tests
│   ├── test_camera.py              # PTZ gimbal kinematics & slew rate tests
│   ├── test_configurable_camera.py # Multi-resolution and FOV audit tests
│   ├── test_ground_truth.py        # Telemetry data integrity tests
│   ├── test_engine.py              # End-to-end simulation loop tests
│   ├── test_detector.py            # Classical CV detector tests (shapes, sizes, distractors)
│   ├── test_centroid.py            # Intensity-weighted subpixel centroid moment tests
│   ├── test_kalman.py              # 4D Kalman filter estimation, velocity, coasting tests
│   ├── test_state_machine.py       # Tracking FSM transitions and loss/reacquisition tests
│   ├── test_pipeline.py            # Integrated VisionPipeline tests on moving sequences
│   ├── test_evaluator.py           # Objective metric computation & strict isolation tests
│   └── test_visualizer.py          # Visualizer panel layout and rendering tests
├── main.py                         # Simulation & Tracking CLI runner & live telemetry
├── run_benchmark.py                # Standalone benchmark execution script
├── requirements.txt                # Dependencies (numpy, opencv-python, fastapi, uvicorn, pytest)
├── PROJECT_CONTEXT.md              # Complete project context, SIH specs, and roadmap
├── docs/
│   └── FRONTEND_ARCHITECTURE.md    # Frontend & API architecture and contract documentation
└── README.md                       # Documentation and usage instructions
```

---

## 3. Mathematical Principles & Projection

1. **Angular Resolution / Pixel Scale**:
   $$\text{Scale}_x = \frac{\text{FOV}_h}{W_{\text{cam}}} = \frac{4.0^\circ}{640} = 0.00625^\circ/\text{px} = 1.09083 \times 10^{-4}\text{ rad/px}$$
   $$\text{Scale}_y = \frac{\text{FOV}_v}{H_{\text{cam}}} = \frac{3.0^\circ}{480} = 0.00625^\circ/\text{px} = 1.09083 \times 10^{-4}\text{ rad/px}$$

2. **World to Angular Bore-Sight Error**:
   $$\Delta \theta_{\text{az}} = (X_b - X_{\text{cam}}) \cdot \text{Scale}_x - \theta_{\text{pan}}$$
   $$\Delta \theta_{\text{el}} = (Y_b - Y_{\text{cam}}) \cdot \text{Scale}_y - \theta_{\text{tilt}}$$

3. **Sensor Plane Projection**:
   $$u = u_0 + \frac{\Delta \theta_{\text{az}}}{\text{Scale}_x} = u_0 + (X_b - X_{\text{cam}}) - \frac{\theta_{\text{pan}}}{\text{Scale}_x}$$
   $$v = v_0 + \frac{\Delta \theta_{\text{el}}}{\text{Scale}_y} = v_0 + (Y_b - Y_{\text{cam}}) - \frac{\theta_{\text{tilt}}}{\text{Scale}_y}$$
   where $(u_0, v_0) = (320.0, 240.0)$ is the optical principal point.

4. **Intensity-Weighted Subpixel Centroid**:
   $$u_c = \frac{m_{10}}{m_{00}} = \frac{\sum_u \sum_v u \cdot I(u, v)}{\sum_u \sum_v I(u, v)}, \quad v_c = \frac{m_{01}}{m_{00}} = \frac{\sum_u \sum_v v \cdot I(u, v)}{\sum_u \sum_v I(u, v)}$$

5. **Kalman State Vector**:
   $$\mathbf{x}_k = [u_k, v_k, \dot{u}_k, \dot{v}_k]^T, \quad \mathbf{z}_k = [u_m, v_m]^T$$

---

## 4. Installation & Setup

### Requirements:
- Python 3.11+
- CPU-compatible (no GPU required for Phase 1-4)

```bash
# Clone or navigate to the repository
cd 26169backend

# Install dependencies
pip install -r requirements.txt
```

---

## 5. Running the Developer Web Frontend & API Server

### Launch the Full Developer Application:
```bash
python -m uvicorn backend.api.server:app --host 127.0.0.1 --port 8000 --reload
```
Open `http://127.0.0.1:8000/` in any modern web browser to access:
- **World Frustum Visualization**: Real-time 2000×2000 space, FOV frustum, and trajectory trails.
- **Sensor Viewport**: 640×480 monochrome sensor frame with Ground Truth, Raw CV (`✕`), and Kalman (`◯`) reticles.
- **Live Telemetry & Tracking HUD**: Dynamic FPS, CV computation latency ($< 1\text{ ms}$), instant and cumulative RMSE, and tracking state.
- **Real-Time Error Graph & State Ribbon**: Dynamic canvas chart plotting Raw vs. Kalman error over time and FSM state transitions.
- **Interactive Controls & Benchmarks**: Runtime configuration tuning, scenario execution, and Phase 3 PTZ interface placeholder.

Interactive Swagger API docs: `http://127.0.0.1:8000/docs`

---

## 6. Running Desktop Visualizer & CLI

### Desktop OpenCV Visualizer:
```bash
python visualizer.py --motion circular
python visualizer.py --motion figure_eight
python visualizer.py --motion random
```

### Headless CLI Demonstration:
```bash
python main.py --motion circular --steps 60
```

---

## 7. Running Unit Tests & Benchmark Matrix

### Run Complete Test Suite (89 Tests):
```bash
pytest -v
```

### Run Standalone Benchmark Runner:
```bash
python run_benchmark.py
```

---

## 8. Phased Development Roadmap

- [x] **Phase 1: Foundational Simulation Engine & Mathematical Models**
- [x] **Phase 2: Classical CV Detection, Subpixel Centroiding & Kalman Tracking**
- [x] **Integration: FastAPI Service Layer & Developer Frontend Interface**
- [ ] **Phase 3: Closed-Loop Gimbal Control (PID/PD pan/tilt tracking)**
- [ ] **Phase 4: Environmental Disturbances & Atmospheric Noise**
- [ ] **Phase 5: Lightweight AI Beacon Verification**
- [ ] **Phase 6: Automated Performance Benchmarking**
- [ ] **Phase 7: Evaluator MP4 Mode**


