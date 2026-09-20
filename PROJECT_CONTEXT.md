# Project Context: AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals

**Smart India Hackathon (SIH) 2026**  
**Problem Statement ID**: SIH26169  
**Organization**: Department of Space / ISRO  
**Category**: Software  
**Theme**: Smart Automation / Space Technology  
**Team**: Greedy Minds  

---

## 1. Problem Statement & Operational Objective

### Background:
Free Space Optical Communication (FSOC) systems use extremely narrow optical laser beams (often milliradian to sub-milliradian divergence) to achieve high data rates over line-of-sight propagation channels. In mobile scenarios (e.g., satellite-to-ground, UAV-to-ground, inter-satellite, vehicle-to-infrastructure), platform movement, vibration, and atmospheric disturbances cause misalignment. 

Before fine pointing (using fast steering mirrors / quadrant detectors) can operate, a **coarse alignment system** must:
1. Observe the angular search space with a wide/medium Field of View (FOV) optical camera.
2. Detect and acquire the optical beacon emitted by the remote terminal.
3. Accurately estimate the beacon's centroid in image coordinates.
4. Calculate pointing error relative to the optical bore-sight.
5. Command a virtual/physical Pan-Tilt mechanism to center the beacon within the camera FOV.
6. Maintain lock despite platform dynamics, jitter, and environmental noise.

### Software-Only Simulation Requirement:
This project delivers a **100% software-based virtual camera tracking, simulation, and benchmarking suite**. It replaces costly physical optical benches, laser emitters, and gimbal hardware with a high-fidelity virtual simulation environment, allowing researchers and evaluators to test, validate, and benchmark coarse alignment algorithms under realistic optical conditions.

---

## 2. Official Problem Statement Specifications & Parameters

### Virtual Camera Parameters:
- **Virtual Scene Dimensions**: Minimum $2000 \times 2000$ pixels.
- **Camera Resolution**: $640 \times 480$ pixels (default).
- **Camera Field of View (FOV)**: $4.0^\circ$ (horizontal) $\times 3.0^\circ$ (vertical) (default).
- **Pixel-to-Angle Scale**: 
  $$S_x = \frac{4.0^\circ}{640} = 0.00625^\circ/\text{px} = 1.09083 \times 10^{-4}\text{ rad/px}$$
  $$S_y = \frac{3.0^\circ}{480} = 0.00625^\circ/\text{px} = 1.09083 \times 10^{-4}\text{ rad/px}$$
- **Initial Camera Position**: Center of the virtual world $(1000, 1000)$.
- **Initial Pointing**: Directed toward world center $(0^\circ \text{ pan}, 0^\circ \text{ tilt})$.
- **Camera Update Rate**: Minimum 30 Hz ($\Delta t = 1/30 \text{ s}$).
- **Sensor Model**: Monochrome 8-bit sensor (color optional).

### Mathematical Coordinate Systems & Angular Projection Model:
- **Implementation Abstraction**:
  The SIH problem statement specifies a virtual scene of minimum $2000 \times 2000$ pixels and a virtual camera with default $640 \times 480$ resolution and $4.0^\circ \times 3.0^\circ$ FOV. To model this in software without requiring an arbitrary 3D distance/depth parameter $D$, the simulation adopts a **calibrated 2D angular search-plane abstraction**:
  - The world coordinate system $(X_w, Y_w)$ is defined as a 2D angular search plane ($2000 \times 2000$ px default).
  - Each world unit maps to an angular increment determined by the camera's resolution and field of view:
    $$S_x = \frac{\text{FOV}_h}{W_{cam}}, \quad S_y = \frac{\text{FOV}_v}{H_{cam}}$$
    For the default configuration ($640 \times 480$, $4.0^\circ \times 3.0^\circ$), $S_x = S_y = 0.00625^\circ/\text{px} = 1.09083 \times 10^{-4}\text{ rad/px}$, giving a total search field of $12.5^\circ \times 12.5^\circ$.
  - In a true physical 3D camera, the spatial footprint on a target plane at depth $D$ is $W = 2D \tan(\text{FOV}_h/2) \approx D \cdot \text{FOV}_h$. The 2D angular search-plane abstraction is mathematically equivalent to a depth-normalized plane (where $1\text{ world pixel} = 1\text{ sensor pixel}$ at zero gimbal deflection). The camera's instantaneous FOV therefore observes a $W_{cam} \times H_{cam}$ window of the $2000 \times 2000$ search space.
  - This design is an intentional, self-contained software abstraction that provides exact, reversible, and configurable angular projection for any resolution $(W_{cam}, H_{cam})$ and any FOV $(\text{FOV}_h, \text{FOV}_v)$.
- **General Projection Equations**:
  For arbitrary camera resolution $(W_{cam}, H_{cam})$, field of view $(\text{FOV}_h, \text{FOV}_v)$, principal point $(u_0, v_0) = (W_{cam}/2, H_{cam}/2)$, camera position $(X_{cam}, Y_{cam})$ with pan $\theta_{pan}$ and tilt $\theta_{tilt}$, and beacon position $(X_b, Y_b)$:
  $$\Delta \theta_{az} = (X_b - X_{cam}) \cdot S_x - \theta_{pan}$$
  $$\Delta \theta_{el} = (Y_b - Y_{cam}) \cdot S_y - \theta_{tilt}$$
  $$u = u_0 + \frac{\Delta \theta_{az}}{S_x} = \frac{W_{cam}}{2} + (X_b - X_{cam}) - \frac{\theta_{pan}}{S_x}$$
  $$v = v_0 + \frac{\Delta \theta_{el}}{S_y} = \frac{H_{cam}}{2} + (Y_b - Y_{cam}) - \frac{\theta_{tilt}}{S_y}$$
- **Verification Example (Default Config)**:
  With camera at $(1000.0, 1000.0)$, $\theta_{pan} = 0^\circ, \theta_{tilt} = 0^\circ$, and beacon at $(1043.1, 861.4)$:
  $$\Delta X = 1043.1 - 1000.0 = +43.1 \implies u = 320.0 + 43.1 = 363.1\text{ px}$$
  $$\Delta Y = 861.4 - 1000.0 = -138.6 \implies v = 240.0 - 138.6 = 101.4\text{ px}$$
  $$\Delta \theta_{az} = +43.1 \times 0.00625^\circ = +0.269375^\circ, \quad \Delta \theta_{el} = -138.6 \times 0.00625^\circ = -0.866250^\circ$$

### Target / Optical Beacon Parameters:
- **Target Type**: Optical beacon spot (laser/LED beacon emission).
- **Target Multiplicity**: 1 mandatory (primary beacon), multiple targets supported optionally.
- **Target Shape**: Configurable (Square default, 2D Gaussian beam spot / Airy disk optional).
- **Target Dimensions**: $5 \times 5$ px to $20 \times 20$ px (Default: $10 \times 10$ px).
- **Initial Location**: Configurable / Random within world bounds.
- **Mandatory Motion Models**:
  1. Straight Line (constant velocity vector with boundary reflection)
  2. Circular (orbiting center point with configurable radius and angular rate)
  3. Figure of Eight (Lemniscate trajectory with dual-axis frequency harmonic)
  4. Random Walk / Filtered Drift (Ornstein-Uhlenbeck stochastic acceleration)
- **Optional Motion Models**: Spiral, Sinusoidal, User-defined waypoint path.

### Gimbal / Pan-Tilt Kinematics:
- **Maximum Pan Speed**: Configurable, approximately $5^\circ/\text{s} - 10^\circ/\text{s}$.
- **Maximum Tilt Speed**: Configurable, approximately $5^\circ/\text{s} - 10^\circ/\text{s}$.
- **Update Rate**: $\ge 20 \text{ Hz}$ (Engine runs at 30 Hz default).

### Performance Metrics (Targeted in Benchmarking):
- **Acquisition Time**: $\le 2.0 \text{ s}$
- **Tracking Error**: $\le 10 \text{ px}$ (coarse alignment threshold)
- **Target Loss**: $< 5\%$
- **Re-acquisition Time**: $\le 1.0 \text{ s}$
- **Processing Speed**: $\ge 20 \text{ FPS}$

### Disturbance & Environmental Noise (Phased Roadmap):
- **Sensor Noise**: Salt & Pepper, Additive Gaussian, Poisson shot noise.
- **Atmospheric Degradation**: Clear, Haze, Fog, Rain attenuation, Low light scintillation.
- **Platform Dynamics**: Platform jitter, Linear vehicle motion, Angular vibration.

### Evaluator Mode:
- Support for evaluator-provided MP4 video playback at $\sim 30 \text{ FPS}$, bypassing virtual PTZ camera motion while running the exact detection, tracking, and evaluation pipeline.

---

## 3. High-Level Closed-Loop Architecture

```
+-------------------------------------------------------------------------------+
|                             VIRTUAL ENVIRONMENT                               |
|                                                                               |
|   +-------------------+    World Position    +----------------------------+   |
|   |   Moving Beacon   | -------------------> |    Virtual Camera Model    |   |
|   | (Motion Models)   |                      | (640x480, 4°x3° FOV, PTZ)  |   |
|   +-------------------+                      +----------------------------+   |
|                                                            |                  |
+------------------------------------------------------------|------------------+
                                                             v
                                               +----------------------------+
                                               | Noise & Atmospheric Dist.  | (Phase 4)
                                               +----------------------------+
                                                             |
                                                             v
                                               +----------------------------+
                                               | Classical Candidate Detect | (Phase 2)
                                               +----------------------------+
                                                             |
                                                             v
                                               +----------------------------+
                                               | AI Beacon Verification     | (Phase 5)
                                               +----------------------------+
                                                             |
                                                             v
                                               +----------------------------+
                                               | Centroid Estimation        | (Phase 2)
                                               +----------------------------+
                                                             |
                                                             v
                                               +----------------------------+
                                               | Kalman State Filtering     | (Phase 2)
                                               +----------------------------+
                                                             |
                                                             v
                                               +----------------------------+
                                               | Tracking State Machine     | (Phase 2)
                                               +----------------------------+
                                                             |
                                                             v
                                               +----------------------------+
                                               | Pixel -> Angular Error     | (Phase 3)
                                               +----------------------------+
                                                             |
                                                             v
                                               +----------------------------+
                                               | Closed-Loop PID Controller | (Phase 3)
                                               +----------------------------+
                                                             |
                                              Pan/Tilt Rate Limited Commands
                                                             |
                                                             v
                                                [ Updates Camera Pointing ]
```

---

## 4. Technology Stack

- **Runtime**: Python 3.11
- **Core Numerical Computing**: NumPy $\ge 1.24$
- **Computer Vision**: OpenCV (`opencv-python`)
- **State Estimation**: FilterPy / NumPy Kalman formulation
- **Deep Learning (Phase 5)**: PyTorch (CPU/GPU compatible)
- **Unit Testing**: PyTest
- **Hardware Profile**: CPU-compatible (tested on AMD Ryzen 5 8645HS / 16GB RAM / RTX 3050).

---

## 5. Architectural Principles

1. **Frontend-Agnostic Core**:
   The simulation and tracking core has zero dependency on any GUI or frontend framework (Qt, OpenGL, React, etc.). Clean state dataclasses, telemetry packets, and frame arrays are passed via pure Python APIs.
2. **Authoritative Ground Truth Separation**:
   Ground truth calculations (exact world coordinates, true image projection, true visibility, bore-sight error) are computed directly from the simulation models and kept strictly separate from estimated/tracked states.
3. **Reproducibility & Modularity**:
   Every component (World, Beacon, Camera, Motion, Projection, Renderer, State) is isolated in dedicated modules with explicit configuration dataclasses.
4. **Physical & Mathematical Consistency**:
   Angular scale, field-of-view, perspective projection, and rate limits are derived from first-principles trigonometry rather than arbitrary pixel transformations.

---

## 6. Phased Implementation Roadmap

- [x] **Phase 1: Foundational Simulation Engine & Development Visualizer**
  - Virtual 2000×2000 world environment
  - Optical beacon model (shape, size, intensity)
  - 4 Motion models: Straight-line, Circular, Figure-of-8, Random
  - Virtual 640×480 camera with 4°×3° FOV & pan/tilt kinematics
  - Rigorous World-to-Camera angular projection & FOV visibility clipping
  - Ground-truth state generation & telemetry
  - 8-bit monochrome frame renderer
  - Frontend-agnostic engine API & comprehensive unit test suite
  - Lightweight real-time (30 Hz) Development Visualizer (`visualizer.py` / `main.py --view`) showing synchronized World View, Sensor View, and Telemet- [x] **Phase 1: Foundational Simulation Engine & Development Visualizer**
  - Virtual 2000×2000 world environment
  - Optical beacon model (shape, size, intensity)
  - 4 Motion models: Straight-line, Circular, Figure-of-8, Random
  - Virtual 640×480 camera with 4°×3° FOV & pan/tilt kinematics
  - Rigorous World-to-Camera angular projection & FOV visibility clipping
  - Ground-truth state generation & telemetry
  - 8-bit monochrome frame renderer
  - Frontend-agnostic engine API & comprehensive unit test suite
  - Lightweight real-time (30 Hz) Development Visualizer (`visualization/viewer.py` / `main.py --view`) showing synchronized World View, Sensor View, and Telemetry HUD
- [x] **Phase 2: Classical Computer Vision Detection, Centroiding & Basic Temporal Tracking**
  - Modular classical CV detector (`backend/vision/detector.py`) with configurable thresholding, contour extraction, geometric/photometric filtering, and deterministic candidate scoring
  - High-precision sub-pixel centroid estimation (`backend/vision/centroid.py`) using intensity-weighted spatial moments ($m_{10}/m_{00}, m_{01}/m_{00}$)
  - 4D Constant-Velocity Kalman Filter (`backend/vision/kalman.py`) operating on $[u, v, \dot{u}, \dot{v}]^T$ with continuous white-noise acceleration $Q(\Delta t)$ and missing-frame coasting
  - 5-State Tracking Finite State Machine (`backend/vision/state_machine.py`) with `SEARCHING`, `ACQUIRED`, `TRACKING`, `LOST`, `REACQUIRED` states
  - Unified `VisionPipeline` (`backend/vision/pipeline.py`) operating purely on raw image frames ($0\%$ ground truth access)
  - Independent `TrackingEvaluator` (`backend/evaluation/evaluator.py`) measuring RMSE, detection success rate, target loss, and acquisition latency against authoritative ground truth
  - Real-time visual tracking telemetry overlay on Camera Sensor View (Ground Truth reticle, Raw Detected crosshair, Kalman Filtered ring & velocity vector, Live HUD status)
- [ ] **Phase 3: Closed-Loop Gimbal Control**
  - Image space to angular error transformation
  - Discrete-time PID / PD controller with anti-windup
  - Pan/tilt velocity and acceleration limiting
  - Dynamic closed-loop camera orientation updates
- [ ] **Phase 4: Environmental Disturbances & Sensor Noise**
  - Additive Gaussian, Poisson, and Salt-and-Pepper sensor noise models
  - Atmospheric transmission models (Fog, Haze, Scintillation, Attenuation)
  - Platform vibration & jitter dynamics
- [ ] **Phase 5: Lightweight AI Verification**
  - Dataset generator for synthetic beacon patches vs. noise/distractors
  - Lightweight CNN / MobileNet-derived binary verifier
  - Hybrid pipeline integration (Classical CV candidates $\to$ AI verifier)
  - Comparative benchmark: CV-only vs. CV+AI
- [ ] **Phase 6: Benchmarking & Automated Performance Telemetry**
  - Acquisition time, RMSE, Lock retention, Target loss, Re-acquisition time
  - High-precision telemetry logging and automated benchmarking reports
- [ ] **Phase 7: Evaluator MP4 Mode**
  - Evaluator video ingest pipeline
  - Virtual PTZ bypass & tracking validation on external footage

---

## 7. Current Implementation Status & Vision Architecture

**Current Milestone**: Phase 2 Completed & Validated (76/76 passing tests).

### Vision Pipeline Architecture (Strict Isolation):
```
Virtual Simulation Engine
         │
  [Raw 8-bit Frame] (np.ndarray: 640x480)
         │  (NO Ground Truth / NO Simulation State)
         ▼
┌─────────────────────────────────────────────────────────────┐
│                       VisionPipeline                        │
│                                                             │
│  1. BeaconDetector:                                         │
│     Grayscale Frame → Binary Thresholding → Contour Extract │
│     → Area/Aspect Filter → Distance/Intensity Scoring       │
│                                                             │
│  2. Intensity-Weighted Subpixel Centroid:                   │
│     m10/m00, m01/m00 on candidate bounding box              │
│                                                             │
│  3. 2D/4D Kalman Filter:                                    │
│     State: [u, v, u_dot, v_dot]^T                           │
│     Prediction & Measurement Update / Coasting              │
│                                                             │
│  4. Tracking State Machine:                                 │
│     SEARCHING → ACQUIRED → TRACKING → LOST → REACQUIRED     │
└─────────────────────────────────────────────────────────────┘
         │
  [TrackerOutput] (raw_detection, filtered_centroid, state)
         │
         ├────────────────────────────────────────┐
         ▼                                        ▼
┌───────────────────────────────┐ ┌───────────────────────────────┐
│     TrackingEvaluator         │ │     SimulationVisualizer      │
│  Compares TrackerOutput with  │ │  Renders World Frustum,       │
│  authoritative GroundTruthState│ │  Camera Sensor Overlays, and  │
│  Computes RMSE, loss, latency │ │  Real-Time Tracking HUD       │
└───────────────────────────────┘ └───────────────────────────────┘
```

### Performance & Centroid Accuracy (Post-Audit):
- **Gaussian Beacon Profile**: Mean raw centroid error $< 0.01\text{ px}$ (RMSE $< 0.01\text{ px}$); Kalman Filtered RMSE $\sim 0.14\text{ px}$.
- **Square Beacon Profile**: Mean raw centroid error $\sim 0.74\text{ px}$; Kalman Filtered RMSE $\sim 0.64\text{ px}$ (Kalman actively attenuates rasterization quantization noise).
- **Noisy Measurement Suppression**: Reduces measurement noise by $\sim 50-70\%$ across linear trajectories ($\sigma_n = 1.0\text{ px} \to \text{RMSE}_{KF} \approx 0.52\text{ px}$).
- **Pipeline Processing Speed**: $\sim 0.5 - 0.8\text{ ms/frame}$ ($> 1200\text{ FPS}$ on standard CPU), comfortably exceeding the 30 Hz / 20 FPS SIH requirement.
- **Strict Isolation**: Programmatically verified via `test_strict_vision_isolation` that no ground-truth attributes or simulation state leaks into the vision detector or tracker.

### Kalman Filter Benchmark Audit Summary:
1. **Mathematical Mechanics**: Under clean synthetic simulations with smooth Gaussian spots, raw subpixel centroiding achieves near-infinite SNR (error $< 0.01\text{ px}$). A 2D Constant-Velocity (CV) model on high-curvature trajectories (e.g., $R=150\text{ px}$ circular motion with centripetal acceleration $a_c \approx 92.5\text{ px/s}^2$) exhibits steady-state geometric lag when over-smoothed with low bandwidth ($q=15, r=1.0$).
2. **Calibrated Operating Point**: Calibrated `KalmanConfig(process_noise_std=80.0, measurement_noise_std=0.2)` provides balanced bandwidth: eliminating geometric curvature lag ($< 0.15\text{ px}$ on circular, $< 0.40\text{ px}$ on figure-8) while providing $> 45\%$ noise reduction on noisy measurements, robust 5-frame coasting extrapolation, and sub-pixel velocity convergence ($\sim 0.2\text{ px/s}$ RMSE).

