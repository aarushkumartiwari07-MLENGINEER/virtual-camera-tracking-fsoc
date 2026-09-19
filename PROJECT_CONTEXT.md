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
In alignment with the software-only problem statement:
- **World Space ($X_w, Y_w$)**: A 2D planar angular tracking space of $2000 \times 2000$ pixels.
- **Angular Mapping**: Each world pixel represents an angular increment of:
  $$S_x = \frac{\text{FOV}_h}{W_{cam}} = \frac{4.0^\circ}{640} = 0.00625^\circ/\text{px} = 1.09083 \times 10^{-4}\text{ rad/px}$$
  $$S_y = \frac{\text{FOV}_v}{H_{cam}} = \frac{3.0^\circ}{480} = 0.00625^\circ/\text{px} = 1.09083 \times 10^{-4}\text{ rad/px}$$
  The full $2000 \times 2000$ virtual search plane corresponds to a total angular uncertainty region of $12.5^\circ \times 12.5^\circ$ ($2000 \times 0.00625^\circ$).
- **Depth / 3D Physical Footprint Relationship**:
  In a physical 3D optical system, spatial footprint at distance $D$ is $W = 2D \tan(\text{FOV}_h/2) \approx D \cdot \text{FOV}_h$. In this software simulation, world coordinates operate in the **calibrated angular search plane** (equivalent to depth-normalized plane where $1\text{ world unit} = 1\text{ sensor pixel}$ at zero pan/tilt). Thus, the instantaneous camera FOV observes exactly a $640 \times 480$ world-unit window of the $2000 \times 2000$ angular search space.
- **Projection Equations**:
  For camera position $(X_{cam}, Y_{cam})$ with pan $\theta_{pan}$ and tilt $\theta_{tilt}$, target position $(X_b, Y_b)$, and principal point $(u_0, v_0) = (320.0, 240.0)$:
  $$\Delta \theta_{az} = (X_b - X_{cam}) \cdot S_x - \theta_{pan}$$
  $$\Delta \theta_{el} = (Y_b - Y_{cam}) \cdot S_y - \theta_{tilt}$$
  $$u = u_0 + \frac{\Delta \theta_{az}}{S_x} = 320.0 + (X_b - X_{cam}) - \frac{\theta_{pan}}{S_x}$$
  $$v = v_0 + \frac{\Delta \theta_{el}}{S_y} = 240.0 + (Y_b - Y_{cam}) - \frac{\theta_{tilt}}{S_y}$$
- **Verification Example**:
  With camera at $(1000.0, 1000.0)$, $\theta_{pan} = 0^\circ, \theta_{tilt} = 0^\circ$, and beacon at $(1043.1, 861.4)$:
  $$\Delta X = 1043.1 - 1000.0 = +43.1 \implies u = 320.0 + 43.1 = 363.1\text{ px}$$
  $$\Delta Y = 861.4 - 1000.0 = -138.6 \implies v = 240.0 - 138.6 = 101.4\text{ px}$$
  $$\Delta \theta_{az} = +43.1 \times 0.00625^\circ = +0.269^\circ, \quad \Delta \theta_{el} = -138.6 \times 0.00625^\circ = -0.866^\circ$$

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
  - Lightweight real-time (30 Hz) Development Visualizer (`visualizer.py` / `main.py --view`) showing synchronized World View, Sensor View, and Telemetry HUD
- [ ] **Phase 2: Classical Tracking Pipeline**
  - Candidate beacon detection (thresholding, contour analysis, morphological ops)
  - Sub-pixel centroid estimation (intensity-weighted moments)
  - 2D/4D Kalman filter for position/velocity estimation
  - Multi-state tracking FSM (Acquiring, Tracking, Coasting, Lost)
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

## 7. Current Implementation Status & Development Visualizer

**Current Milestone**: Phase 1 Completed & Visually Verified.
- All core simulation models, mathematical projections, motion trajectories, ground-truth data models, and sensor renderer implemented.
- Pure Python simulation engine tested with full test coverage (24/24 passing unit tests).
- **Phase 1 Debug Visualizer** (`visualizer.py`): Real-time (~30 Hz) OpenCV-based development viewer:
  1. **World View Panel** ($520 \times 520$ px): Renders the $2000 \times 2000$ world, coordinate grid, camera position marker, dynamic camera FOV frustum footprint, beacon trajectory history trail, instantaneous velocity vector, and current beacon position.
  2. **Camera Sensor View Panel** ($640 \times 480$ px): Displays the actual raw monochrome sensor frame produced by `SimulationEngine.renderer`, overlaid with optical center bore-sight crosshairs, target acquisition marker, subpixel centroid coordinates, and "OUT OF FOV" alert banners.
  3. **Live Telemetry & Control HUD**: Real-time readout of simulation time, frame counter, world coordinates, pan/tilt gimbal angles, visibility status, image pixel coordinates $(u, v)$, and bore-sight error vectors.
  4. **Interactive Controls**: `[SPACE]` Play/Pause, `[R]` Reset, `[1-4]` Switch motion model on-the-fly, `[S/F]` Adjust simulation speed, `[Q/ESC]` Exit.
- **Important Note**: This viewer is strictly a Phase 1 internal development and debug validation tool, completely decoupled from the core engine and not the final evaluator GUI. It visually validates physical kinematics, angular projections, and sensor rasterization before implementing Phase 2 tracking.

