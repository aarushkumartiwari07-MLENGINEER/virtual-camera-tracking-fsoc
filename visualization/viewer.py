"""
Real-time development visualizer and debug viewer for SIH26169 Phase 1.
Renders side-by-side World View, Camera Sensor View, and Telemetry HUD.
Zero modification to the underlying engine; pure presentation layer.
"""

import time
import math
from typing import Tuple, List, Optional, Dict, Any
import numpy as np
import cv2

from backend.config.simulation_config import SimulationConfig, BeaconConfig
from backend.config.vision_config import VisionConfig
from backend.engine import SimulationEngine
from backend.models.state import GroundTruthState
from backend.models.tracking_state import TrackingState, TrackerOutput
from backend.vision.pipeline import VisionPipeline
from backend.evaluation.evaluator import TrackingEvaluator


class SimulationVisualizer:
    """
    Real-time interactive viewer for the FSOC Virtual Camera simulation and Phase 2 tracking.
    Consumes SimulationEngine output through the standalone VisionPipeline and displays:
    1. World View (2000x2000 scene with FOV frustum, camera pos, and beacon trail)
    2. Camera View (Monochrome sensor frame with Ground Truth, Raw Detected, and Kalman Filtered markers)
    3. Live Ground-Truth vs Tracking Telemetry HUD & Interactive Controls
    """

    def __init__(
        self,
        engine: Optional[SimulationEngine] = None,
        config: Optional[SimulationConfig] = None,
        vision_config: Optional[VisionConfig] = None,
        target_fps: float = 30.0,
        trail_length: int = 120,
    ) -> None:
        self.config = config or SimulationConfig(fps=target_fps)
        self.vision_config = vision_config or VisionConfig()
        self.engine = engine or SimulationEngine(self.config)
        self.pipeline = VisionPipeline(config=self.vision_config)
        self.evaluator = TrackingEvaluator()

        self.target_fps = float(target_fps)
        self.frame_delay_ms = max(1, int(round(1000.0 / self.target_fps)))
        self.speed_multiplier: float = 1.0

        # Trail history of beacon world positions [(x, y), ...]
        self.trail_length = trail_length
        self.beacon_trail: List[Tuple[float, float]] = []

        # Interactive state
        self.is_paused: bool = False
        self.current_motion: str = self.config.beacon.motion_type

        # UI Layout dimensions
        self.world_panel_size = 520  # 520x520 square for world view
        self.cam_width = self.engine.camera.width
        self.cam_height = self.engine.camera.height
        self.hud_height = 200
        self.margin = 15

        # Canvas total size
        self.total_width = self.world_panel_size + self.cam_width + 3 * self.margin
        self.total_height = max(self.world_panel_size, self.cam_height) + self.hud_height + 3 * self.margin

        self.window_name = "SIH26169 FSOC Virtual Camera Tracking - Phase 2 Vision Viewer (Greedy Minds)"

    def reset(self, motion_type: Optional[str] = None) -> None:
        """Reset simulation, vision pipeline, evaluator, and clear trajectory trail."""
        if motion_type is not None:
            self.current_motion = motion_type
            self.config.beacon.motion_type = motion_type
            # Set appropriate default parameters for selected motion
            if motion_type == "circular":
                self.config.beacon.motion_params = {
                    "radius": 180.0,
                    "angular_speed_deg_s": 45.0,
                }
            elif motion_type == "figure_eight":
                self.config.beacon.motion_params = {
                    "amplitude_x": 220.0,
                    "amplitude_y": 140.0,
                    "period_s": 7.0,
                }
            elif motion_type == "straight_line":
                self.config.beacon.motion_params = {
                    "speed": 60.0,
                    "angle_deg": 35.0,
                    "bounce": True,
                }
            elif motion_type == "random":
                self.config.beacon.motion_params = {
                    "max_speed": 70.0,
                    "acceleration_std": 45.0,
                }
            self.engine = SimulationEngine(self.config)

        self.engine.reset()
        self.pipeline.reset()
        self.evaluator.reset()
        self.beacon_trail.clear()

    def world_to_panel_coords(
        self,
        xw: float,
        yw: float,
        world_w: float,
        world_h: float,
    ) -> Tuple[int, int]:
        """Map world space coordinates [0, Ww]x[0, Hw] to panel pixels [0, Pw]x[0, Ph]."""
        px = int(round((xw / world_w) * (self.world_panel_size - 1)))
        py = int(round((yw / world_h) * (self.world_panel_size - 1)))
        return px, py

    def render_world_view(self, state: GroundTruthState) -> np.ndarray:
        """
        Draw the 2000x2000 virtual world representation.
        Shows world borders, grid lines, camera FOV frustum footprint,
        camera position, beacon trajectory trail, and current beacon position.
        """
        panel = np.zeros((self.world_panel_size, self.world_panel_size, 3), dtype=np.uint8)
        panel[:] = (20, 24, 28)  # Deep dark slate background

        world_w = self.engine.world.width
        world_h = self.engine.world.height

        # 1. Draw subtle coordinate grid (every 500 world units)
        grid_step = 500.0
        for gx in np.arange(grid_step, world_w, grid_step):
            px, _ = self.world_to_panel_coords(gx, 0, world_w, world_h)
            cv2.line(panel, (px, 0), (px, self.world_panel_size), (40, 48, 56), 1, cv2.LINE_AA)
            cv2.putText(panel, f"{int(gx)}", (px + 3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 95, 110), 1)

        for gy in np.arange(grid_step, world_h, grid_step):
            _, py = self.world_to_panel_coords(0, gy, world_w, world_h)
            cv2.line(panel, (0, py), (self.world_panel_size, py), (40, 48, 56), 1, cv2.LINE_AA)
            cv2.putText(panel, f"{int(gy)}", (4, py - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 95, 110), 1)

        # World outer boundary
        cv2.rectangle(panel, (0, 0), (self.world_panel_size - 1, self.world_panel_size - 1), (70, 85, 100), 1)

        # 2. Compute and draw Camera FOV Frustum Footprint in World Space
        cam = self.engine.camera
        scale_x = cam.scale_x_deg_px
        scale_y = cam.scale_y_deg_px
        fov_center_x = state.camera_world_pos[0] + (state.camera_pan_deg / scale_x)
        fov_center_y = state.camera_world_pos[1] + (state.camera_tilt_deg / scale_y)

        half_w = cam.width / 2.0
        half_h = cam.height / 2.0

        fov_min_x = fov_center_x - half_w
        fov_min_y = fov_center_y - half_h
        fov_max_x = fov_center_x + half_w
        fov_max_y = fov_center_y + half_h

        p_min = self.world_to_panel_coords(fov_min_x, fov_min_y, world_w, world_h)
        p_max = self.world_to_panel_coords(fov_max_x, fov_max_y, world_w, world_h)

        # Draw semi-transparent FOV footprint
        fov_overlay = panel.copy()
        fov_color = (0, 180, 255) if state.is_visible else (60, 100, 180)
        cv2.rectangle(fov_overlay, p_min, p_max, fov_color, -1)
        cv2.addWeighted(fov_overlay, 0.15, panel, 0.85, 0, panel)
        cv2.rectangle(panel, p_min, p_max, fov_color, 2, cv2.LINE_AA)

        # 3. Draw Camera Position & Bore-Sight Axis
        cam_p = self.world_to_panel_coords(state.camera_world_pos[0], state.camera_world_pos[1], world_w, world_h)
        cv2.drawMarker(panel, cam_p, (255, 200, 50), cv2.MARKER_DIAMOND, 14, 2, cv2.LINE_AA)
        
        fov_c_p = self.world_to_panel_coords(fov_center_x, fov_center_y, world_w, world_h)
        cv2.line(panel, cam_p, fov_c_p, (255, 200, 50), 1, cv2.LINE_AA)

        # 4. Draw Beacon Trajectory Trail
        if len(self.beacon_trail) > 1:
            for idx in range(1, len(self.beacon_trail)):
                pt1 = self.world_to_panel_coords(self.beacon_trail[idx - 1][0], self.beacon_trail[idx - 1][1], world_w, world_h)
                pt2 = self.world_to_panel_coords(self.beacon_trail[idx][0], self.beacon_trail[idx][1], world_w, world_h)
                alpha = idx / float(len(self.beacon_trail))
                trail_color = (int(40 * alpha), int(220 * alpha), int(120 * alpha))
                cv2.line(panel, pt1, pt2, trail_color, 1, cv2.LINE_AA)

        # 5. Draw Current Beacon Target Position
        beacon_p = self.world_to_panel_coords(state.beacon_world_pos[0], state.beacon_world_pos[1], world_w, world_h)
        b_color = (0, 255, 120) if state.is_visible else (0, 100, 255)
        cv2.circle(panel, beacon_p, 6, b_color, -1, cv2.LINE_AA)
        cv2.circle(panel, beacon_p, 10, b_color, 1, cv2.LINE_AA)

        vx, vy = state.beacon_world_velocity
        vel_end = (
            int(round(beacon_p[0] + vx * 0.2)),
            int(round(beacon_p[1] + vy * 0.2)),
        )
        cv2.arrowedLine(panel, beacon_p, vel_end, (0, 255, 255), 1, cv2.LINE_AA, tipLength=0.3)

        # Panel Header
        cv2.rectangle(panel, (0, 0), (self.world_panel_size, 26), (15, 18, 22), -1)
        cv2.putText(panel, f"WORLD VIEW [{int(world_w)}x{int(world_h)} px]", (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Cam @ ({int(state.camera_world_pos[0])},{int(state.camera_world_pos[1])})", (self.world_panel_size - 170, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1, cv2.LINE_AA)

        return panel

    def render_camera_view(
        self,
        raw_frame: np.ndarray,
        state: GroundTruthState,
        tracker_output: Optional[TrackerOutput] = None,
    ) -> np.ndarray:
        """
        Overlay Phase 2 Vision tracking annotations on the raw camera sensor frame:
        - Ground Truth: Yellow circle (labeled GT)
        - Raw Detection: Cyan crosshair (labeled RAW)
        - Kalman Filtered: Magenta ring & velocity vector (labeled KF)
        """
        view = cv2.cvtColor(raw_frame, cv2.COLOR_GRAY2BGR)

        # Optical Bore-Sight Crosshair at Principal Point (u0, v0)
        cx = int(round(self.engine.camera.principal_point[0]))
        cy = int(round(self.engine.camera.principal_point[1]))
        cross_color = (80, 120, 160)
        cv2.line(view, (cx - 20, cy), (cx + 20, cy), cross_color, 1, cv2.LINE_AA)
        cv2.line(view, (cx, cy - 20), (cx, cy + 20), cross_color, 1, cv2.LINE_AA)
        cv2.circle(view, (cx, cy), 15, cross_color, 1, cv2.LINE_AA)

        # Sensor frame border
        cv2.rectangle(view, (0, 0), (self.cam_width - 1, self.cam_height - 1), (80, 80, 80), 1)

        # 1. Draw Ground Truth Marker (Yellow Reticle)
        if state.is_visible and state.beacon_image_pos is not None:
            gt_u, gt_v = state.beacon_image_pos
            gt_iu, gt_iv = int(round(gt_u)), int(round(gt_v))
            cv2.circle(view, (gt_iu, gt_iv), 8, (0, 220, 255), 1, cv2.LINE_AA)
            cv2.putText(view, "GT", (gt_iu + 10, gt_iv - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 220, 255), 1, cv2.LINE_AA)

        # 2. Draw Raw Detection Marker (Cyan Box & Cross)
        if tracker_output is not None and tracker_output.raw_detection.is_detected:
            det = tracker_output.raw_detection
            if det.centroid is not None:
                du, dv = det.centroid
                diu, div = int(round(du)), int(round(dv))
                
                if det.bbox is not None:
                    b_umin, b_vmin, b_umax, b_vmax = det.bbox
                    cv2.rectangle(view, (b_umin, b_vmin), (b_umax, b_vmax), (255, 255, 0), 1, cv2.LINE_AA)

                cv2.drawMarker(view, (diu, div), (255, 255, 0), cv2.MARKER_CROSS, 10, 1, cv2.LINE_AA)
                cv2.putText(view, f"RAW: ({du:.1f},{dv:.1f})", (diu + 10, div + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 0), 1, cv2.LINE_AA)

        # 3. Draw Kalman Filtered Estimate (Magenta Ring & Velocity)
        if tracker_output is not None and tracker_output.filtered_centroid is not None:
            fu, fv = tracker_output.filtered_centroid
            fiu, fiv = int(round(fu)), int(round(fv))

            kf_color = (0, 180, 255) if tracker_output.is_predicted else (255, 0, 255)  # Amber if coasting, Magenta if locked
            cv2.circle(view, (fiu, fiv), 12, kf_color, 1, cv2.LINE_AA)

            if tracker_output.filtered_velocity is not None:
                fvx, fvy = tracker_output.filtered_velocity
                vel_end = (int(round(fiu + fvx * 0.15)), int(round(fiv + fvy * 0.15)))
                cv2.arrowedLine(view, (fiu, fiv), vel_end, kf_color, 1, cv2.LINE_AA, tipLength=0.3)

            tag = "KF(pred)" if tracker_output.is_predicted else "KF"
            cv2.putText(view, f"{tag}: ({fu:.1f},{fv:.1f})", (fiu - 35, fiv - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.36, kf_color, 1, cv2.LINE_AA)

        # 4. Out of FOV Banner
        if not state.is_visible:
            banner_h = 40
            by1 = max(0, cy - banner_h // 2)
            by2 = min(self.cam_height, cy + banner_h // 2)
            banner = np.zeros((by2 - by1, self.cam_width, 3), dtype=np.uint8)
            banner[:] = (0, 0, 160)
            cv2.addWeighted(banner, 0.6, view[by1:by2, :], 0.4, 0, view[by1:by2, :])
            cv2.putText(
                view,
                "** TARGET OUTSIDE CAMERA FOV **",
                (max(10, self.cam_width // 2 - 165), cy + 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # Header Bar
        cv2.rectangle(view, (0, 0), (self.cam_width, 26), (15, 18, 22), -1)
        fov_h, fov_v = self.engine.camera.fov_deg
        cv2.putText(
            view,
            f"CAMERA SENSOR VIEW [{self.cam_width}x{self.cam_height} | {fov_h:.1f}deg x {fov_v:.1f}deg FOV]",
            (10, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 220, 255),
            1,
            cv2.LINE_AA,
        )
        
        # Tracking FSM State Badge
        fsm_state = tracker_output.state if tracker_output is not None else TrackingState.SEARCHING
        if fsm_state == TrackingState.TRACKING:
            badge_col = (0, 255, 100) # Green
        elif fsm_state == TrackingState.ACQUIRED:
            badge_col = (255, 255, 0) # Cyan
        elif fsm_state == TrackingState.REACQUIRED:
            badge_col = (255, 0, 255) # Magenta
        elif fsm_state == TrackingState.SEARCHING:
            badge_col = (0, 200, 255) # Yellow/Amber
        else:
            badge_col = (0, 60, 255)  # Red
        
        cv2.putText(view, f"TRACK: {fsm_state.value}", (self.cam_width - 165, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, badge_col, 1, cv2.LINE_AA)

        return view

    def render_hud_dashboard(
        self,
        state: GroundTruthState,
        tracker_output: Optional[TrackerOutput] = None,
        summary_metrics: Optional[Dict[str, Any]] = None,
    ) -> np.ndarray:
        """
        Draw live telemetry readout comparing Ground Truth against Phase 2 Vision Tracker.
        """
        hud = np.zeros((self.hud_height, self.total_width, 3), dtype=np.uint8)
        hud[:] = (16, 20, 24)

        cv2.rectangle(hud, (0, 0), (self.total_width - 1, self.hud_height - 1), (50, 60, 72), 1)

        # Title bar
        cv2.rectangle(hud, (0, 0), (self.total_width, 24), (24, 30, 38), -1)
        cv2.putText(
            hud,
            "REAL-TIME SIMULATION & PHASE 2 CLASSICAL CV TRACKING TELEMETRY",
            (15, 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (0, 220, 255),
            1,
            cv2.LINE_AA,
        )

        pause_status = " [PAUSED]" if self.is_paused else " [RUNNING]"
        pause_color = (0, 180, 255) if self.is_paused else (0, 255, 100)
        cv2.putText(
            hud,
            f"STATE:{pause_status}  SPEED: {self.speed_multiplier:.1f}x",
            (self.total_width - 240, 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            pause_color,
            1,
            cv2.LINE_AA,
        )

        # Column 1: World Kinematics & Camera
        col1_x = 20
        y = 46
        dy = 20
        cv2.putText(hud, f"Sim Time:     {state.timestamp:6.3f} s  (Frame #{state.frame_index})", (col1_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 210, 220), 1)
        cv2.putText(hud, f"Motion Model: {self.current_motion.upper()}", (col1_x, y + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1)
        cv2.putText(hud, f"Beacon World: ({state.beacon_world_pos[0]:6.1f}, {state.beacon_world_pos[1]:6.1f}) px", (col1_x, y + 2 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 210, 220), 1)
        cv2.putText(hud, f"Beacon Speed: ({state.beacon_world_velocity[0]:+5.1f}, {state.beacon_world_velocity[1]:+5.1f}) px/s", (col1_x, y + 3 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 210, 220), 1)
        cv2.putText(hud, f"Camera Pan:   {state.camera_pan_deg:+.2f} deg | Tilt: {state.camera_tilt_deg:+.2f} deg", (col1_x, y + 4 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 210, 220), 1)

        # Column 2: Phase 2 Computer Vision Tracking State
        col2_x = 380
        fsm_state = tracker_output.state if tracker_output is not None else TrackingState.SEARCHING
        cv2.putText(hud, f"FSM State:      {fsm_state.value}", (col2_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)

        det_str = "DETECTED" if (tracker_output and tracker_output.raw_detection.is_detected) else ("COASTING (pred)" if (tracker_output and tracker_output.is_predicted) else "NO DETECTION")
        det_col = (0, 255, 120) if (tracker_output and tracker_output.raw_detection.is_detected) else ((0, 180, 255) if (tracker_output and tracker_output.is_predicted) else (0, 80, 255))
        cv2.putText(hud, f"Detection:      {det_str}", (col2_x, y + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, det_col, 1)

        raw_pos_str = f"({tracker_output.raw_detection.centroid[0]:.1f}, {tracker_output.raw_detection.centroid[1]:.1f})" if (tracker_output and tracker_output.raw_detection.centroid) else "N/A"
        cv2.putText(hud, f"Raw Centroid:   {raw_pos_str}", (col2_x, y + 2 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 210, 220), 1)

        kf_pos_str = f"({tracker_output.filtered_centroid[0]:.1f}, {tracker_output.filtered_centroid[1]:.1f})" if (tracker_output and tracker_output.filtered_centroid) else "N/A"
        cv2.putText(hud, f"Kalman Pos:     {kf_pos_str}", (col2_x, y + 3 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 120, 255), 1)

        # Column 3: Performance & Evaluation Metrics (GT Comparison)
        col3_x = 720
        raw_err_str = "N/A"
        filt_err_str = "N/A"
        if state.is_visible and state.beacon_image_pos is not None and tracker_output is not None:
            gt_u, gt_v = state.beacon_image_pos
            if tracker_output.raw_detection.centroid is not None:
                du = tracker_output.raw_detection.centroid[0] - gt_u
                dv = tracker_output.raw_detection.centroid[1] - gt_v
                raw_err_str = f"{math.hypot(du, dv):.2f} px"
            if tracker_output.filtered_centroid is not None:
                fdu = tracker_output.filtered_centroid[0] - gt_u
                fdv = tracker_output.filtered_centroid[1] - gt_v
                filt_err_str = f"{math.hypot(fdu, fdv):.2f} px"

        cv2.putText(hud, f"Raw Error vs GT:    {raw_err_str}", (col3_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 0), 1)
        cv2.putText(hud, f"Kalman Error vs GT: {filt_err_str}", (col3_x, y + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 120, 255), 1)

        det_rate = summary_metrics.get("detection_rate_pct", 0.0) if summary_metrics else 0.0
        cv2.putText(hud, f"Detection Rate:     {det_rate:.1f}%", (col3_x, y + 2 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 100), 1)

        proc_ms = tracker_output.total_processing_time_ms if tracker_output else 0.0
        fps = (1000.0 / proc_ms) if proc_ms > 0 else 0.0
        cv2.putText(hud, f"Vision Latency/FPS: {proc_ms:.2f} ms ({fps:.0f} FPS)", (col3_x, y + 3 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 210, 220), 1)

        # Column 4: Controls Guide
        col4_x = 1000
        cv2.putText(hud, "CONTROLS:", (col4_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1)
        cv2.putText(hud, "[SPACE] Play/Pause", (col4_x, y + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (180, 190, 200), 1)
        cv2.putText(hud, "[R]     Reset", (col4_x, y + 2 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (180, 190, 200), 1)
        cv2.putText(hud, "[1-4]   1=Line 2=Circ 3=8 4=Rnd", (col4_x, y + 3 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (180, 190, 200), 1)
        cv2.putText(hud, "[S / F] Slower / Faster", (col4_x, y + 4 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (180, 190, 200), 1)

        return hud

    def compose_dashboard(self, world_panel: np.ndarray, camera_view: np.ndarray, hud_panel: np.ndarray) -> np.ndarray:
        """Combine World View, Camera View, and HUD into a single composite frame."""
        canvas = np.zeros((self.total_height, self.total_width, 3), dtype=np.uint8)
        canvas[:] = (12, 14, 18)  # Deep canvas background

        top_height = max(self.world_panel_size, self.cam_height)

        # Place World View at top-left (vertically centered in top section)
        wx = self.margin
        wy = self.margin + (top_height - self.world_panel_size) // 2
        canvas[wy:wy + self.world_panel_size, wx:wx + self.world_panel_size] = world_panel

        # Place Camera View at top-right (vertically centered in top section)
        cx = wx + self.world_panel_size + self.margin
        cy = self.margin + (top_height - self.cam_height) // 2
        canvas[cy:cy + self.cam_height, cx:cx + self.cam_width] = camera_view

        # Place HUD at bottom
        hy = self.margin + top_height + self.margin
        canvas[hy:hy + self.hud_height, self.margin:self.margin + self.total_width - 2 * self.margin] = hud_panel[:, self.margin:self.total_width - self.margin]

        return canvas

    def step_and_render(self) -> Tuple[np.ndarray, GroundTruthState, TrackerOutput]:
        """Advance one simulation step, run vision pipeline & evaluator, and return composite dashboard."""
        if not self.is_paused:
            dt = (1.0 / self.target_fps) * self.speed_multiplier
            raw_frame, gt_state = self.engine.step(dt=dt)
            self.beacon_trail.append(gt_state.beacon_world_pos)
            if len(self.beacon_trail) > self.trail_length:
                self.beacon_trail.pop(0)

            # Process frame through standalone Phase 2 vision pipeline
            tracker_output = self.pipeline.process_frame(
                frame=raw_frame,
                dt=dt,
                timestamp=gt_state.timestamp,
                frame_index=gt_state.frame_index,
            )

            # Evaluate step
            self.evaluator.evaluate_step(tracker_output=tracker_output, ground_truth=gt_state)

        else:
            gt_state = self.engine.get_state()
            raw_frame = self.engine.renderer.render(
                self.engine.beacon, self.engine.camera, gt_state.is_visible, gt_state.beacon_image_pos
            )
            tracker_output = self.pipeline.last_output or self.pipeline.process_frame(
                frame=raw_frame,
                dt=0.0,
                timestamp=gt_state.timestamp,
                frame_index=gt_state.frame_index,
            )

        summary_metrics = self.evaluator.get_summary_metrics()

        world_panel = self.render_world_view(gt_state)
        camera_view = self.render_camera_view(raw_frame, gt_state, tracker_output)
        hud_panel = self.render_hud_dashboard(gt_state, tracker_output, summary_metrics)
        dashboard = self.compose_dashboard(world_panel, camera_view, hud_panel)

        return dashboard, gt_state, tracker_output

    def run(self) -> None:
        """
        Main interactive loop for real-time visualization at 30 Hz.
        """
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        print("[*] Starting Phase 1 Simulation Visualizer...")
        print("[*] Controls: [SPACE] Pause/Play | [R] Reset | [1-4] Motion Selection | [S/F] Speed | [Q] Quit")

        try:
            while True:
                t0 = time.perf_counter()

                dashboard, state = self.step_and_render()
                cv2.imshow(self.window_name, dashboard)

                # Compute wait time to maintain ~30 Hz
                elapsed_ms = (time.perf_counter() - t0) * 1000.0
                wait_time = max(1, int(round(self.frame_delay_ms - elapsed_ms)))
                key = cv2.waitKey(wait_time) & 0xFF

                if key in (ord('q'), ord('Q'), 27):  # 'q' or ESC
                    print("[*] Exiting visualizer.")
                    break
                elif key == 32:  # Spacebar
                    self.is_paused = not self.is_paused
                elif key in (ord('r'), ord('R')):
                    self.reset()
                elif key == ord('1'):
                    self.reset("straight_line")
                elif key == ord('2'):
                    self.reset("circular")
                elif key == ord('3'):
                    self.reset("figure_eight")
                elif key == ord('4'):
                    self.reset("random")
                elif key in (ord('f'), ord('F')):  # Faster
                    self.speed_multiplier = min(5.0, self.speed_multiplier + 0.25)
                elif key in (ord('s'), ord('S')):  # Slower
                    self.speed_multiplier = max(0.25, self.speed_multiplier - 0.25)

                # Check if window closed via X button
                if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break

        finally:
            cv2.destroyAllWindows()


def launch_visualizer(motion: str = "circular", fps: float = 30.0) -> None:
    """Convenience entrypoint to launch the visualizer."""
    config = SimulationConfig(fps=fps)
    config.beacon.motion_type = motion
    visualizer = SimulationVisualizer(config=config, target_fps=fps)
    visualizer.run()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="SIH26169 Phase 1 Simulation Visualizer")
    parser.add_argument("--motion", type=str, default="circular", choices=["straight_line", "circular", "figure_eight", "random"])
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()
    launch_visualizer(motion=args.motion, fps=args.fps)
