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

## 2. Project Architecture (Phase 1)

```
26169backend/
├── backend/
│   ├── __init__.py
│   ├── engine.py                   # Master frontend-agnostic SimulationEngine API
│   ├── config/
│   │   ├── __init__.py
│   │   └── simulation_config.py    # Dataclasses for World, Beacon, Camera, Simulation
│   ├── models/
│   │   ├── __init__.py
│   │   └── state.py                # Authoritative GroundTruthState definition
│   └── simulation/
│       ├── __init__.py
│       ├── world.py                # 2000x2000 virtual world & boundary constraints
│       ├── beacon.py               # Optical beacon entity
│       ├── motion.py               # Straight-line, Circular, Figure-8, Random motion models
│       ├── camera.py               # Virtual camera with PTZ gimbal kinematics & rate limits
│       ├── projection.py           # Mathematical World -> Angle -> Pixel projection
│       └── renderer.py             # 8-bit monochrome sensor frame rasterizer
├── tests/
│   ├── __init__.py
│   ├── test_projection.py          # Projection math, FOV clipping, invertibility tests
│   ├── test_motion.py              # Motion trajectory & boundary behavior tests
│   ├── test_camera.py              # PTZ gimbal kinematics & slew rate tests
│   ├── test_ground_truth.py        # Telemetry data integrity tests
│   └── test_engine.py              # End-to-end simulation loop tests
├── main.py                         # Standalone simulation CLI demo runner
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

## 5. Running the Simulation

### Running the CLI Demonstration:
```bash
# Run 60 simulation steps with circular motion
python main.py --motion circular --steps 60

# Run with figure-of-8 motion and save the rendered sensor frame
python main.py --motion figure_eight --steps 90 --save-frame

# Run with random walk drift and Gaussian beam profile
python main.py --motion random --shape gaussian --steps 120
```

### Programmatic Usage (Frontend Agnostic):
```python
from backend.engine import SimulationEngine
from backend.config import SimulationConfig, BeaconConfig, CameraConfig

# Configure simulation
config = SimulationConfig(fps=30.0)
config.beacon.motion_type = "figure_eight"

# Instantiate engine
engine = SimulationEngine(config)

# Run discrete steps
for _ in range(100):
    frame, ground_truth = engine.step()
    # frame: np.ndarray of shape (480, 640), dtype uint8
    # ground_truth: GroundTruthState dataclass (exact world/pixel coords, bore-sight error)
    if ground_truth.is_visible:
        print(f"Beacon at image coords: {ground_truth.beacon_image_pos}")
```

---

## 6. Running Unit Tests

Run the complete test suite using pytest:
```bash
pytest -v
```

---

## 7. Current Limitations & Next Steps

### Implemented in Phase 1:
- Virtual 2000x2000 environment.
- Beacon entity with Square, Circle, and Gaussian optical spot rendering.
- 4 Motion models: Straight line, Circular, Figure of 8, Random walk.
- Virtual 640x480 camera with 4°x3° FOV and 2-axis PTZ gimbal kinematics.
- Mathematical projection and FOV frustum visibility calculation.
- Separation of authoritative ground-truth state from tracker data.
- Frontend-agnostic Python API.

### Planned for Phase 2:
- Classical OpenCV candidate beacon detection (adaptive thresholding, contour extraction).
- Sub-pixel centroid estimation (intensity-weighted moments).
- Kalman filter state estimation (position and velocity filtering).
- Coarse alignment finite state machine (Acquiring, Tracking, Coasting, Lost).
