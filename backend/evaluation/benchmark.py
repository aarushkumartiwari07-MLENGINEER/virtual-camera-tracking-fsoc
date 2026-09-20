"""
Comprehensive Benchmark Suite for Phase 2 Classical CV Detection, Centroiding, and Kalman Tracking.
Evaluates tracking performance across controlled motion trajectories, noise levels, and detection dropouts.
Strictly decoupled from simulation internals: feeds frames/measurements and compares with ground truth.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Tuple
import math
import numpy as np

from ..config.simulation_config import SimulationConfig, BeaconConfig, CameraConfig, WorldConfig
from ..config.vision_config import VisionConfig, KalmanConfig
from ..models.detection import CandidateBlob, DetectionResult
from ..models.tracking_state import TrackerOutput
from ..engine import SimulationEngine
from ..vision.pipeline import VisionPipeline
from ..vision.kalman import BeaconKalmanFilter
from ..evaluation.evaluator import TrackingEvaluator


@dataclass
class BenchmarkScenarioResult:
    """Detailed results for a single benchmark scenario."""
    scenario_name: str
    motion_type: str
    shape: str
    noise_std_px: float
    dropout_frames: int
    total_frames: int
    raw_mean_err_px: float
    raw_rmse_px: float
    raw_max_err_px: float
    filtered_mean_err_px: float
    filtered_rmse_px: float
    filtered_max_err_px: float
    velocity_rmse_px_s: Optional[float]
    acquisition_time_s: Optional[float]
    coasting_max_err_px: Optional[float]
    reacquisition_latency_frames: Optional[int]
    notes: str


class TrackingBenchmarkRunner:
    """
    Executes reproducible benchmark scenarios against the VisionPipeline and BeaconKalmanFilter.
    """

    def __init__(self, fps: float = 30.0, steps: int = 120, seed: int = 42) -> None:
        self.fps = fps
        self.dt = 1.0 / fps
        self.steps = steps
        self.seed = seed

    def run_scenario(
        self,
        name: str,
        motion_type: str = "circular",
        shape: str = "gaussian",
        motion_params: Optional[Dict[str, Any]] = None,
        vision_config: Optional[VisionConfig] = None,
        noise_std_px: float = 0.0,
        dropout_interval: Optional[Tuple[int, int]] = None,
        notes: str = "",
    ) -> BenchmarkScenarioResult:
        """
        Run a single benchmark scenario.
        
        Args:
            name: Human-readable scenario name
            motion_type: 'straight_line', 'circular', 'figure_eight', 'random' (or 'static')
            shape: 'gaussian', 'square', 'circle'
            motion_params: Motion parameters dict
            vision_config: Vision configuration to test
            noise_std_px: Synthetic measurement noise std injected into raw detection
            dropout_interval: (start_frame, end_frame) where frames are blanked to simulate loss
            notes: Engineering observations
        """
        rng = np.random.RandomState(self.seed)

        # Handle static scenario by overriding straight line with zero speed
        is_static = (motion_type == "static")
        effective_motion = "straight_line" if is_static else motion_type
        default_params = {
            "radius": 150.0,
            "angular_speed_deg_s": 45.0,
            "amplitude_x": 200.0,
            "amplitude_y": 120.0,
            "period_s": 6.0,
            "speed": 0.0 if is_static else 50.0,
            "angle_deg": 30.0,
        }
        if motion_params:
            default_params.update(motion_params)

        sim_cfg = SimulationConfig(
            fps=self.fps,
            world=WorldConfig(width=2000.0, height=2000.0),
            beacon=BeaconConfig(
                initial_pos=(1000.0, 1000.0),
                size=10.0,
                shape=shape,
                intensity=255.0,
                motion_type=effective_motion,
                motion_params=default_params,
            ),
            camera=CameraConfig(
                resolution=(640, 480),
                fov_deg=(4.0, 3.0),
                initial_pos=(1000.0, 1000.0),
            ),
        )

        engine = SimulationEngine(sim_cfg)
        pipeline = VisionPipeline(vision_config or VisionConfig())

        raw_errors: List[float] = []
        filtered_errors: List[float] = []
        vel_errors: List[float] = []
        coasting_errors: List[float] = []

        dropout_start, dropout_end = dropout_interval if dropout_interval else (-1, -1)
        reacq_counter: Optional[int] = None
        reacq_frames: Optional[int] = None
        acquisition_time: Optional[float] = None

        prev_gt_pos: Optional[Tuple[float, float]] = None

        for frame_idx in range(1, self.steps + 1):
            frame, gt_state = engine.step()

            # True beacon image velocity from finite difference
            gt_pos = gt_state.beacon_image_pos
            gt_vel_u, gt_vel_v = 0.0, 0.0
            if prev_gt_pos is not None and gt_pos is not None:
                gt_vel_u = (gt_pos[0] - prev_gt_pos[0]) / self.dt
                gt_vel_v = (gt_pos[1] - prev_gt_pos[1]) / self.dt
            prev_gt_pos = gt_pos

            # Simulate detection dropout by blanking frame if in dropout range
            is_dropout = (dropout_start <= frame_idx <= dropout_end)
            input_frame = np.zeros_like(frame) if is_dropout else frame

            # Process frame through pure vision pipeline
            tracker_output = pipeline.process_frame(
                frame=input_frame,
                dt=self.dt,
                timestamp=gt_state.timestamp,
                frame_index=frame_idx,
            )

            # If synthetic measurement noise is requested, create perturbed detection and update Kalman filter
            if noise_std_px > 0.0 and tracker_output.raw_detection.is_detected and tracker_output.raw_detection.selected_candidate is not None:
                orig_cand = tracker_output.raw_detection.selected_candidate
                orig_u, orig_v = orig_cand.centroid
                noisy_u = orig_u + float(rng.normal(0.0, noise_std_px))
                noisy_v = orig_v + float(rng.normal(0.0, noise_std_px))
                
                noisy_cand = CandidateBlob(
                    centroid=(noisy_u, noisy_v),
                    bbox=orig_cand.bbox,
                    area=orig_cand.area,
                    peak_intensity=orig_cand.peak_intensity,
                    mean_intensity=orig_cand.mean_intensity,
                    aspect_ratio=orig_cand.aspect_ratio,
                    circularity=orig_cand.circularity,
                    score=orig_cand.score,
                )
                noisy_detection = DetectionResult(
                    is_detected=True,
                    selected_candidate=noisy_cand,
                    all_candidates=tracker_output.raw_detection.all_candidates,
                    frame_index=tracker_output.raw_detection.frame_index,
                    timestamp=tracker_output.raw_detection.timestamp,
                    processing_time_ms=tracker_output.raw_detection.processing_time_ms,
                )
                # Re-update Kalman with noisy measurement for fair noisy filtering benchmark
                fu, fv = pipeline.kalman.update((noisy_u, noisy_v))
                tracker_output = TrackerOutput(
                    state=tracker_output.state,
                    is_tracking=tracker_output.is_tracking,
                    raw_detection=noisy_detection,
                    filtered_centroid=(fu, fv),
                    filtered_velocity=pipeline.kalman.get_velocity(),
                    is_predicted=tracker_output.is_predicted,
                    consecutive_detections=tracker_output.consecutive_detections,
                    consecutive_losses=tracker_output.consecutive_losses,
                    timestamp=tracker_output.timestamp,
                    frame_index=tracker_output.frame_index,
                    total_processing_time_ms=tracker_output.total_processing_time_ms,
                )

            # Record errors if beacon is within visible FOV
            if gt_state.is_visible and gt_pos is not None:
                # Raw Error
                if tracker_output.raw_detection.is_detected and tracker_output.raw_detection.centroid is not None:
                    ru, rv = tracker_output.raw_detection.centroid
                    r_err = math.hypot(ru - gt_pos[0], rv - gt_pos[1])
                    raw_errors.append(r_err)
                    if acquisition_time is None:
                        acquisition_time = gt_state.timestamp

                    if is_dropout:
                        pass
                    elif frame_idx > dropout_end and reacq_counter is not None:
                        # Count frames to re-lock
                        reacq_frames = frame_idx - dropout_end
                        reacq_counter = None

                # Filtered Error
                if tracker_output.filtered_centroid is not None:
                    fu, fv = tracker_output.filtered_centroid
                    f_err = math.hypot(fu - gt_pos[0], fv - gt_pos[1])
                    filtered_errors.append(f_err)

                    if is_dropout:
                        coasting_errors.append(f_err)

                # Velocity Error
                if tracker_output.filtered_velocity is not None and frame_idx > 5:
                    vu, vv = tracker_output.filtered_velocity
                    v_err = math.hypot(vu - gt_vel_u, vv - gt_vel_v)
                    vel_errors.append(v_err)

            if is_dropout and reacq_counter is None and dropout_interval is not None:
                reacq_counter = 0

        # Compute summary metrics
        raw_mean = float(np.mean(raw_errors)) if raw_errors else 0.0
        raw_rmse = float(np.sqrt(np.mean(np.square(raw_errors)))) if raw_errors else 0.0
        raw_max = float(np.max(raw_errors)) if raw_errors else 0.0

        filt_mean = float(np.mean(filtered_errors)) if filtered_errors else 0.0
        filt_rmse = float(np.sqrt(np.mean(np.square(filtered_errors)))) if filtered_errors else 0.0
        filt_max = float(np.max(filtered_errors)) if filtered_errors else 0.0

        vel_rmse = float(np.sqrt(np.mean(np.square(vel_errors)))) if vel_errors else None
        coasting_max = float(np.max(coasting_errors)) if coasting_errors else None

        dropout_count = (dropout_end - dropout_start + 1) if dropout_interval else 0

        return BenchmarkScenarioResult(
            scenario_name=name,
            motion_type=motion_type,
            shape=shape,
            noise_std_px=noise_std_px,
            dropout_frames=dropout_count,
            total_frames=self.steps,
            raw_mean_err_px=raw_mean,
            raw_rmse_px=raw_rmse,
            raw_max_err_px=raw_max,
            filtered_mean_err_px=filt_mean,
            filtered_rmse_px=filt_rmse,
            filtered_max_err_px=filt_max,
            velocity_rmse_px_s=vel_rmse,
            acquisition_time_s=acquisition_time,
            coasting_max_err_px=coasting_max,
            reacquisition_latency_frames=reacq_frames,
            notes=notes,
        )

    def run_full_benchmark_matrix(self, vision_config: Optional[VisionConfig] = None) -> List[BenchmarkScenarioResult]:
        """Run the complete standard suite of benchmark scenarios."""
        results: List[BenchmarkScenarioResult] = []

        # A. Static Target
        results.append(self.run_scenario(
            name="A1. Static Beacon (Clean Gaussian)",
            motion_type="static",
            shape="gaussian",
            vision_config=vision_config,
            notes="Evaluates static jitter and zero-velocity stability",
        ))
        results.append(self.run_scenario(
            name="A2. Static Beacon (Clean Square 10x10)",
            motion_type="static",
            shape="square",
            vision_config=vision_config,
            notes="Evaluates rasterization discretization bias on static target",
        ))

        # B. Constant Velocity Linear Motion
        results.append(self.run_scenario(
            name="B1. Linear Motion (50 px/s, Clean Gaussian)",
            motion_type="straight_line",
            shape="gaussian",
            motion_params={"speed": 50.0, "angle_deg": 30.0},
            vision_config=vision_config,
            notes="Matches constant-velocity Kalman model perfectly",
        ))
        results.append(self.run_scenario(
            name="B2. Linear Motion (50 px/s, Clean Square)",
            motion_type="straight_line",
            shape="square",
            motion_params={"speed": 50.0, "angle_deg": 30.0},
            vision_config=vision_config,
            notes="Linear motion with square spot rasterization noise",
        ))

        # C. Circular Motion (Curved / Accelerating)
        results.append(self.run_scenario(
            name="C1. Circular Motion (R=150px, w=45 deg/s, Clean Gaussian)",
            motion_type="circular",
            shape="gaussian",
            motion_params={"radius": 150.0, "angular_speed_deg_s": 45.0},
            vision_config=vision_config,
            notes="Centripetal acceleration a_c ~ 92.5 px/s^2 tests CV lag",
        ))
        results.append(self.run_scenario(
            name="C2. Circular Motion (R=150px, w=45 deg/s, Clean Square)",
            motion_type="circular",
            shape="square",
            motion_params={"radius": 150.0, "angular_speed_deg_s": 45.0},
            vision_config=vision_config,
            notes="Default SIH Phase 2 demo benchmark",
        ))

        # D. Figure-8 Motion
        results.append(self.run_scenario(
            name="D1. Figure-8 Lemniscate (Clean Gaussian)",
            motion_type="figure_eight",
            shape="gaussian",
            motion_params={"amplitude_x": 200.0, "amplitude_y": 120.0, "period_s": 6.0},
            vision_config=vision_config,
            notes="Dynamic curvature reversal test",
        ))

        # E. Fast Motion
        results.append(self.run_scenario(
            name="E1. High-Speed Linear Motion (120 px/s, Clean Gaussian)",
            motion_type="straight_line",
            shape="gaussian",
            motion_params={"speed": 120.0, "angle_deg": 45.0},
            vision_config=vision_config,
            notes="Tests fast target velocity convergence",
        ))

        # F. Controlled Centroid Measurement Noise Sweep
        for noise_sigma in [0.5, 1.0, 2.0, 3.0, 5.0]:
            results.append(self.run_scenario(
                name=f"F. Noisy Linear Motion (sigma={noise_sigma:.1f} px)",
                motion_type="straight_line",
                shape="gaussian",
                motion_params={"speed": 50.0, "angle_deg": 30.0},
                noise_std_px=noise_sigma,
                vision_config=vision_config,
                notes=f"Tests Kalman noise filtering effectiveness at sigma={noise_sigma:.1f}px",
            ))

        # G. Temporary Detection Loss / Coasting
        results.append(self.run_scenario(
            name="G1. Temporary Loss (5 frames dropout on Linear)",
            motion_type="straight_line",
            shape="gaussian",
            motion_params={"speed": 50.0, "angle_deg": 30.0},
            dropout_interval=(40, 44),  # 5 frames dropped
            vision_config=vision_config,
            notes="Evaluates 5-frame coasting & reacquisition on linear track",
        ))
        results.append(self.run_scenario(
            name="G2. Temporary Loss (5 frames dropout on Circular)",
            motion_type="circular",
            shape="gaussian",
            motion_params={"radius": 150.0, "angular_speed_deg_s": 45.0},
            dropout_interval=(40, 44),  # 5 frames dropped
            vision_config=vision_config,
            notes="Evaluates 5-frame coasting tangent drift on circular track",
        ))

        return results
