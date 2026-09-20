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
│       └── evaluator.py            # Independent TrackingEvaluator comparing Tracker to Ground Truth
├── visualization/
│   ├── __init__.py
│   └── viewer.py                   # Real-time visualizer with GT, Raw, Kalman overlays & HUD
├── tests/
│   ├── __init__.py
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
├── requirements.txt                # Dependencies (numpy, opencv-python, pytest)
├── PROJECT_CONTEXT.md              # Complete project context, SIH specs, and roadmap
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

## 5. Running the Simulation & Visualizer

### A. Real-Time Interactive Visualizer
Launch the 30 Hz real-time graphical dashboard showing synchronized World View, Sensor View with Phase 2 Tracking Reticles, and Ground-Truth vs Tracker Telemetry:

```bash
# Launch visualizer with default circular motion
python visualizer.py

# Or launch via main.py CLI
python main.py --view

# Launch with specific motion model
python visualizer.py --motion figure_eight
python visualizer.py --motion random
python visualizer.py --motion straight_line
```

#### Visual Markers on Sensor View:
- **Green Reticle (`+`)**: Ground-Truth Beacon Center (Evaluation overlay only).
- **Orange Cross (`x`)**: Raw Classical CV Detected Centroid $(u_d, v_d)$.
- **Cyan Ring (`o`) & Cyan Vector**: Kalman Filtered Position Estimate $(u_f, v_f)$ and Estimated Velocity Vector $(\dot{u}, \dot{v})$.

---

### B. Headless CLI Demonstration:
```bash
# Run 60 simulation steps with circular motion (prints step table & summary metrics)
python main.py --motion circular --steps 60

# Run with Gaussian beacon shape profile
python main.py --motion circular --shape gaussian --steps 60
```

### C. Programmatic Usage (Frontend Agnostic):
```python
from backend.engine import SimulationEngine
from backend.config import SimulationConfig, VisionConfig
from backend.vision.pipeline import VisionPipeline
from backend.evaluation.evaluator import TrackingEvaluator

# 1. Setup Simulation Engine & Vision Pipeline
engine = SimulationEngine(SimulationConfig(fps=30.0))
pipeline = VisionPipeline(VisionConfig())
evaluator = TrackingEvaluator()

# 2. Simulation and Tracking Loop
for _ in range(100):
    frame, gt_state = engine.step()
    
    # Pure vision tracking (NO ground-truth input)
    tracker_output = pipeline.process_frame(frame, dt=1.0 / 30.0)
    
    # Independent evaluation layer
    step_metrics = evaluator.evaluate_step(tracker_output, gt_state)

# 3. Print Objective Benchmark Summary
summary = evaluator.get_summary_metrics()
print(f"Detection Success Rate: {summary['detection_rate_pct']:.1f}%")
print(f"Raw RMSE Error: {summary['rmse_raw_error_px']:.3f} px")
print(f"Filtered RMSE Error: {summary['rmse_filtered_error_px']:.3f} px")
```

---

## 6. Running Unit Tests

Run the complete 76-test suite using pytest:
```bash
pytest -v
```

---

## 7. Current Limitations & Next Steps

### Implemented in Phase 1 & Phase 2:
- Virtual 2000x2000 environment and dynamic beacon trajectories.
- Virtual PTZ camera with 4°x3° FOV and physical kinematics.
- High-precision intensity-weighted subpixel centroid extraction.
- Deterministic classical CV candidate detection and scoring.
- 4D Constant-Velocity Kalman filter with missing-frame coasting.
- 5-State Tracking FSM (`SEARCHING`, `ACQUIRED`, `TRACKING`, `LOST`, `REACQUIRED`).
- Independent `TrackingEvaluator` for objective performance verification.
- 76/76 passing automated unit and integration tests.

### Current Limitations:
- Single-beacon primary tracking (multi-target discrimination will be enhanced with AI verification in Phase 5).
- Camera is currently stationary; closed-loop pan/tilt tracking control will be added in Phase 3.
- Clean synthetic background; atmospheric disturbances and sensor noise will be introduced in Phase 4.

### Next Step (Phase 3):
- Implement closed-loop Pan-Tilt-Zoom (PTZ) gimbal control (discrete PID/PD controller) to drive tracking error $(u - u_0, v - v_0) \to 0$.

