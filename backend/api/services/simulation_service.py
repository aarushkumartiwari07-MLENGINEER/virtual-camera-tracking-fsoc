"""
Simulation Service Layer.
Coordinates the background simulation loop, vision pipeline, evaluator,
frame encoding, and asynchronous WebSocket telemetry broadcasts.
Thread-safe and decoupled from REST/WebSocket transport handlers.
"""

import asyncio
import base64
import logging
import time
from typing import Optional, Dict, Any, Tuple
import cv2
import numpy as np

from ...config.simulation_config import (
    SimulationConfig,
    WorldConfig,
    BeaconConfig,
    CameraConfig,
)
from ...config.vision_config import (
    VisionConfig,
    DetectorConfig,
    CentroidConfig,
    KalmanConfig,
    StateMachineConfig,
)
from ...engine import SimulationEngine
from ...vision.pipeline import VisionPipeline
from ...evaluation.evaluator import TrackingEvaluator
from ...models.tracking_state import TrackingState
from ..websocket_manager import ws_manager
from ..schemas import MasterConfigModel

logger = logging.getLogger("api.simulation_service")


class SimulationService:
    """
    Core stateful simulation and tracking service.
    Orchestrates the background execution task and provides synchronous control interfaces.
    """

    def __init__(self) -> None:
        self.sim_config = SimulationConfig(fps=30.0)
        self.vision_config = VisionConfig()

        self.engine = SimulationEngine(self.sim_config)
        self.pipeline = VisionPipeline(self.vision_config)
        self.evaluator = TrackingEvaluator()

        self.status: str = "STOPPED"  # 'STOPPED', 'RUNNING', 'PAUSED'
        self.loop_task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

        # Telemetry cache
        self.last_frame: Optional[np.ndarray] = None
        self.last_gt_state: Optional[Any] = None
        self.last_tracker_output: Optional[Any] = None
        self.last_step_metrics: Optional[Dict[str, Any]] = None
        self.last_telemetry_packet: Optional[Dict[str, Any]] = None

        # History buffer for graphs (last 300 points)
        self.error_history: list = []
        self.state_history: list = []
        self.max_history_len: int = 300

        # Performance counters
        self.actual_fps: float = 0.0
        self._frame_count: int = 0
        self._fps_last_time: float = time.perf_counter()

        # Frame streaming configuration
        self.stream_jpeg_frames: bool = True
        self.jpeg_quality: int = 70

    def get_status(self) -> Dict[str, Any]:
        """Return instantaneous system status."""
        gt_vis = bool(self.last_gt_state.is_visible) if self.last_gt_state else False
        track_state = (
            self.last_tracker_output.state.value
            if self.last_tracker_output
            else TrackingState.SEARCHING.value
        )
        return {
            "status": self.status,
            "frame_index": self.engine.frame_index,
            "timestamp": round(self.engine.sim_time, 4),
            "fps": round(self.actual_fps, 1),
            "target_fps": self.sim_config.fps,
            "motion_type": self.sim_config.beacon.motion_type,
            "is_beacon_visible": gt_vis,
            "tracking_state": track_state,
        }

    def get_current_config(self) -> Dict[str, Any]:
        """Return the complete master configuration as a JSON-compatible dict."""
        return {
            "simulation": {
                "fps": self.sim_config.fps,
                "world": {
                    "width": self.sim_config.world.width,
                    "height": self.sim_config.world.height,
                },
                "beacon": {
                    "initial_pos": self.sim_config.beacon.initial_pos,
                    "size": self.sim_config.beacon.size,
                    "shape": self.sim_config.beacon.shape,
                    "intensity": self.sim_config.beacon.intensity,
                    "motion_type": self.sim_config.beacon.motion_type,
                    "motion_params": self.sim_config.beacon.motion_params,
                },
                "camera": {
                    "resolution": self.sim_config.camera.resolution,
                    "fov_deg": self.sim_config.camera.fov_deg,
                    "initial_pos": self.sim_config.camera.initial_pos,
                    "initial_pan_deg": self.sim_config.camera.initial_pan_deg,
                    "initial_tilt_deg": self.sim_config.camera.initial_tilt_deg,
                    "max_pan_speed_deg_s": self.sim_config.camera.max_pan_speed_deg_s,
                    "max_tilt_speed_deg_s": self.sim_config.camera.max_tilt_speed_deg_s,
                },
            },
            "detector": {
                "intensity_threshold": self.vision_config.detector.intensity_threshold,
                "min_area": self.vision_config.detector.min_area,
                "max_area": self.vision_config.detector.max_area,
                "min_aspect_ratio": self.vision_config.detector.min_aspect_ratio,
                "max_aspect_ratio": self.vision_config.detector.max_aspect_ratio,
                "use_adaptive_threshold": self.vision_config.detector.use_adaptive_threshold,
            },
            "centroid": {
                "method": self.vision_config.centroid.method,
                "crop_padding": self.vision_config.centroid.crop_padding,
            },
            "kalman": {
                "process_noise_std": self.vision_config.kalman.process_noise_std,
                "measurement_noise_std": self.vision_config.kalman.measurement_noise_std,
                "initial_error_std": self.vision_config.kalman.initial_error_std,
                "initial_velocity_std": self.vision_config.kalman.initial_velocity_std,
            },
            "state_machine": {
                "consecutive_acquire_frames": self.vision_config.state_machine.consecutive_acquire_frames,
                "max_lost_coasting_frames": self.vision_config.state_machine.max_lost_coasting_frames,
                "consecutive_reacquire_frames": self.vision_config.state_machine.consecutive_reacquire_frames,
            },
        }

    async def update_config(self, config_data: MasterConfigModel) -> None:
        """Apply new configuration and re-initialize engine/pipeline."""
        async with self._lock:
            # Update simulation config
            self.sim_config = SimulationConfig(
                fps=config_data.simulation.fps,
                world=WorldConfig(
                    width=config_data.simulation.world.width,
                    height=config_data.simulation.world.height,
                ),
                beacon=BeaconConfig(
                    initial_pos=config_data.simulation.beacon.initial_pos,
                    size=config_data.simulation.beacon.size,
                    shape=config_data.simulation.beacon.shape,
                    intensity=config_data.simulation.beacon.intensity,
                    motion_type=config_data.simulation.beacon.motion_type,
                    motion_params=config_data.simulation.beacon.motion_params,
                ),
                camera=CameraConfig(
                    resolution=config_data.simulation.camera.resolution,
                    fov_deg=config_data.simulation.camera.fov_deg,
                    initial_pos=config_data.simulation.camera.initial_pos,
                    initial_pan_deg=config_data.simulation.camera.initial_pan_deg,
                    initial_tilt_deg=config_data.simulation.camera.initial_tilt_deg,
                    max_pan_speed_deg_s=config_data.simulation.camera.max_pan_speed_deg_s,
                    max_tilt_speed_deg_s=config_data.simulation.camera.max_tilt_speed_deg_s,
                ),
            )

            # Update vision config
            self.vision_config = VisionConfig(
                detector=DetectorConfig(
                    intensity_threshold=config_data.detector.intensity_threshold,
                    min_area=config_data.detector.min_area,
                    max_area=config_data.detector.max_area,
                    min_aspect_ratio=config_data.detector.min_aspect_ratio,
                    max_aspect_ratio=config_data.detector.max_aspect_ratio,
                    use_adaptive_threshold=config_data.detector.use_adaptive_threshold,
                ),
                centroid=CentroidConfig(
                    method=config_data.centroid.method,
                    crop_padding=config_data.centroid.crop_padding,
                ),
                kalman=KalmanConfig(
                    process_noise_std=config_data.kalman.process_noise_std,
                    measurement_noise_std=config_data.kalman.measurement_noise_std,
                    initial_error_std=config_data.kalman.initial_error_std,
                    initial_velocity_std=config_data.kalman.initial_velocity_std,
                ),
                state_machine=StateMachineConfig(
                    consecutive_acquire_frames=config_data.state_machine.consecutive_acquire_frames,
                    max_lost_coasting_frames=config_data.state_machine.max_lost_coasting_frames,
                    consecutive_reacquire_frames=config_data.state_machine.consecutive_reacquire_frames,
                ),
            )

            # Recreate instances
            self.engine = SimulationEngine(self.sim_config)
            self.pipeline = VisionPipeline(self.vision_config)
            self.evaluator.reset()
            self.error_history.clear()
            self.state_history.clear()

            await ws_manager.broadcast_event(
                category="CONFIG",
                message="Master configuration updated and simulation engine re-initialized.",
                level="INFO",
            )

    async def start(self) -> None:
        """Start or resume background simulation task."""
        async with self._lock:
            if self.status == "RUNNING":
                return

            self.status = "RUNNING"
            if self.loop_task is None or self.loop_task.done():
                self.loop_task = asyncio.create_task(self._simulation_loop())

            await ws_manager.broadcast_event(
                category="SIMULATION",
                message=f"Simulation started at {self.sim_config.fps} Hz (motion={self.sim_config.beacon.motion_type}).",
                level="INFO",
            )
            await ws_manager.broadcast_state_change(self.status, self.get_status()["tracking_state"])

    async def pause(self) -> None:
        """Pause running simulation."""
        async with self._lock:
            if self.status != "RUNNING":
                return
            self.status = "PAUSED"
            await ws_manager.broadcast_event(
                category="SIMULATION",
                message="Simulation paused.",
                level="INFO",
            )
            await ws_manager.broadcast_state_change(self.status, self.get_status()["tracking_state"])

    async def resume(self) -> None:
        """Resume paused simulation."""
        await self.start()

    async def stop(self) -> None:
        """Stop simulation and cancel background loop."""
        async with self._lock:
            self.status = "STOPPED"
            if self.loop_task and not self.loop_task.done():
                self.loop_task.cancel()
                try:
                    await self.loop_task
                except asyncio.CancelledError:
                    pass
                self.loop_task = None

            await ws_manager.broadcast_event(
                category="SIMULATION",
                message="Simulation stopped.",
                level="INFO",
            )
            await ws_manager.broadcast_state_change(self.status, self.get_status()["tracking_state"])

    async def reset(self, motion_type: Optional[str] = None) -> None:
        """Reset simulation engine, vision tracker, and evaluators."""
        async with self._lock:
            if motion_type:
                self.sim_config.beacon.motion_type = motion_type
                self.engine = SimulationEngine(self.sim_config)
            else:
                self.engine.reset()

            self.pipeline.reset()
            self.evaluator.reset()
            self.error_history.clear()
            self.state_history.clear()

            # Execute initial step to generate frame 0
            self._execute_single_step()

            await ws_manager.broadcast_event(
                category="SIMULATION",
                message=f"Simulation reset (motion={self.sim_config.beacon.motion_type}).",
                level="INFO",
            )
            if self.last_telemetry_packet:
                await ws_manager.broadcast_telemetry(self.last_telemetry_packet)

    async def step_frame(self) -> Optional[Dict[str, Any]]:
        """Advance simulation by exactly one frame (for manual step-debugging)."""
        async with self._lock:
            if self.status == "RUNNING":
                self.status = "PAUSED"

            packet = self._execute_single_step()
            if packet:
                await ws_manager.broadcast_telemetry(packet)
            return packet

    def get_metrics_summary(self) -> Dict[str, Any]:
        """Return aggregate summary metrics from TrackingEvaluator."""
        return self.evaluator.get_summary_metrics()

    def get_metrics_history(self) -> Dict[str, Any]:
        """Return history buffer for real-time graphs."""
        return {
            "error_history": self.error_history,
            "state_history": self.state_history,
        }

    # =========================================================================
    # Internal Step & Execution Loop
    # =========================================================================

    def _execute_single_step(self) -> Optional[Dict[str, Any]]:
        """Execute one simulation step, run vision pipeline, and assemble telemetry."""
        dt = 1.0 / self.sim_config.fps

        # 1. Simulation Engine Step
        frame, gt_state = self.engine.step()
        self.last_frame = frame
        self.last_gt_state = gt_state

        # 2. Vision Pipeline Step (Strictly receives ONLY frame and dt)
        tracker_output = self.pipeline.process_frame(
            frame=frame,
            dt=dt,
            timestamp=gt_state.timestamp,
            frame_index=gt_state.frame_index,
        )
        self.last_tracker_output = tracker_output

        # 3. Independent Evaluator Step
        step_metrics = self.evaluator.evaluate_step(tracker_output, gt_state)
        self.last_step_metrics = step_metrics

        # 4. Frame Encoding for WebSocket Streaming
        frame_base64: Optional[str] = None
        if self.stream_jpeg_frames and frame is not None:
            try:
                ret, jpeg_buffer = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                )
                if ret:
                    frame_base64 = base64.b64encode(jpeg_buffer).decode("utf-8")
            except Exception as e:
                logger.warning(f"Failed to encode frame: {e}")

        # 5. Extract Candidate Details
        cand = tracker_output.raw_detection.selected_candidate
        raw_bbox = cand.bbox if cand else None
        raw_area = cand.area if cand else None
        raw_peak = cand.peak_intensity if cand else None

        # 6. Assemble Structured Telemetry Packet
        packet = {
            "frame_index": gt_state.frame_index,
            "timestamp": round(gt_state.timestamp, 4),
            "dt": dt,
            "ground_truth": {
                "beacon_world_pos": (round(gt_state.beacon_world_pos[0], 2), round(gt_state.beacon_world_pos[1], 2)),
                "beacon_world_vel": (round(gt_state.beacon_world_velocity[0], 2), round(gt_state.beacon_world_velocity[1], 2)),
                "beacon_image_pos": (
                    (round(gt_state.beacon_image_pos[0], 2), round(gt_state.beacon_image_pos[1], 2))
                    if gt_state.beacon_image_pos else None
                ),
                "camera_world_pos": (round(gt_state.camera_world_pos[0], 2), round(gt_state.camera_world_pos[1], 2)),
                "camera_pan_deg": round(gt_state.camera_pan_deg, 3),
                "camera_tilt_deg": round(gt_state.camera_tilt_deg, 3),
                "fov_deg": (self.sim_config.camera.fov_deg[0], self.sim_config.camera.fov_deg[1]),
                "is_visible": gt_state.is_visible,
                "boresight_pixel_error": (
                    (round(gt_state.pixel_error[0], 2), round(gt_state.pixel_error[1], 2))
                    if gt_state.pixel_error else None
                ),
                "boresight_angular_error_deg": (
                    (round(gt_state.angular_error_deg[0], 4), round(gt_state.angular_error_deg[1], 4))
                    if gt_state.angular_error_deg else None
                ),
            },
            "tracking": {
                "tracking_state": tracker_output.state.value,
                "is_detected": tracker_output.raw_detection.is_detected,
                "candidate_count": len(tracker_output.raw_detection.all_candidates),
                "raw_centroid": (
                    (round(tracker_output.raw_detection.centroid[0], 2), round(tracker_output.raw_detection.centroid[1], 2))
                    if tracker_output.raw_detection.centroid else None
                ),
                "raw_bbox": raw_bbox,
                "raw_area": round(raw_area, 1) if raw_area is not None else None,
                "raw_peak_intensity": round(raw_peak, 1) if raw_peak is not None else None,
                "filtered_centroid": (
                    (round(tracker_output.filtered_centroid[0], 2), round(tracker_output.filtered_centroid[1], 2))
                    if tracker_output.filtered_centroid else None
                ),
                "filtered_velocity_px_s": (
                    (round(tracker_output.filtered_velocity[0], 2), round(tracker_output.filtered_velocity[1], 2))
                    if tracker_output.filtered_velocity else None
                ),
                "is_predicted": tracker_output.is_predicted,
                "consecutive_detections": tracker_output.consecutive_detections,
                "consecutive_losses": tracker_output.consecutive_losses,
                "processing_time_ms": round(tracker_output.total_processing_time_ms, 2),
            },
            "errors": {
                "raw_error_px": round(step_metrics["raw_error_px"], 3) if step_metrics["raw_error_px"] is not None else None,
                "filtered_error_px": round(step_metrics["filtered_error_px"], 3) if step_metrics["filtered_error_px"] is not None else None,
            },
            "sensor_frame_base64": frame_base64,
        }

        self.last_telemetry_packet = packet

        # 7. Append to graph history buffer
        history_item = {
            "frame_index": gt_state.frame_index,
            "timestamp": gt_state.timestamp,
            "raw_error_px": step_metrics["raw_error_px"],
            "filtered_error_px": step_metrics["filtered_error_px"],
        }
        self.error_history.append(history_item)
        if len(self.error_history) > self.max_history_len:
            self.error_history.pop(0)

        state_item = {
            "frame_index": gt_state.frame_index,
            "timestamp": gt_state.timestamp,
            "state": tracker_output.state.value,
        }
        self.state_history.append(state_item)
        if len(self.state_history) > self.max_history_len:
            self.state_history.pop(0)

        return packet

    async def _simulation_loop(self) -> None:
        """Background asynchronous execution loop running at target FPS."""
        logger.info("Simulation background loop started.")
        target_interval = 1.0 / self.sim_config.fps

        while self.status == "RUNNING":
            t_start = time.perf_counter()

            # Execute single discrete step
            packet = self._execute_single_step()

            # Broadcast live telemetry to all connected WebSocket clients
            if packet:
                await ws_manager.broadcast_telemetry(packet)

            # Update FPS measurement
            self._frame_count += 1
            now = time.perf_counter()
            elapsed_fps = now - self._fps_last_time
            if elapsed_fps >= 1.0:
                self.actual_fps = self._frame_count / elapsed_fps
                self._frame_count = 0
                self._fps_last_time = now

            # Sleep remaining time to maintain target FPS
            step_elapsed = time.perf_counter() - t_start
            sleep_time = max(0.001, target_interval - step_elapsed)
            await asyncio.sleep(sleep_time)

        logger.info("Simulation background loop terminated.")


# Global singleton instance
sim_service = SimulationService()
