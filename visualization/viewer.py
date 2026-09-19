"""
Real-time development visualizer and debug viewer for SIH26169 Phase 1.
Renders side-by-side World View, Camera Sensor View, and Telemetry HUD.
Zero modification to the underlying engine; pure presentation layer.
"""

import time
import math
from typing import Tuple, List, Optional
import numpy as np
import cv2

from backend.config.simulation_config import SimulationConfig, BeaconConfig
from backend.engine import SimulationEngine
from backend.models.state import GroundTruthState


class SimulationVisualizer:
    """
    Real-time interactive viewer for the FSOC Virtual Camera simulation.
    Consumes the real SimulationEngine output and displays:
    1. World View (2000x2000 scene with FOV frustum and beacon trajectory)
    2. Camera View (640x480 actual sensor frame with optical bore-sight reticle)
    3. Live Ground-Truth Telemetry HUD & Interactive Controls
    """

    def __init__(
        self,
        engine: Optional[SimulationEngine] = None,
        config: Optional[SimulationConfig] = None,
        target_fps: float = 30.0,
        trail_length: int = 120,
    ) -> None:
        self.config = config or SimulationConfig(fps=target_fps)
        self.engine = engine or SimulationEngine(self.config)
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
        self.cam_width = 640
        self.cam_height = 480
        self.hud_height = 180
        self.margin = 15

        # Canvas total size
        self.total_width = self.world_panel_size + self.cam_width + 3 * self.margin
        self.total_height = max(self.world_panel_size, self.cam_height) + self.hud_height + 3 * self.margin

        self.window_name = "SIH26169 FSOC Virtual Camera Tracking - Phase 1 Viewer (Greedy Minds)"

    def reset(self, motion_type: Optional[str] = None) -> None:
        """Reset simulation and clear trajectory trail."""
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
        # Center of FOV in world units = Xcam + pan / Scale_x, Ycam + tilt / Scale_y
        cam = self.engine.camera
        scale_x = cam.scale_x_deg_px
        scale_y = cam.scale_y_deg_px
        fov_center_x = state.camera_world_pos[0] + (state.camera_pan_deg / scale_x)
        fov_center_y = state.camera_world_pos[1] + (state.camera_tilt_deg / scale_y)

        # Half size of camera view in world units = Wcam / 2, Hcam / 2
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
        fov_color = (0, 180, 255) if state.is_visible else (60, 100, 180)  # Amber when visible, blue when searching
        cv2.rectangle(fov_overlay, p_min, p_max, fov_color, -1)
        cv2.addWeighted(fov_overlay, 0.15, panel, 0.85, 0, panel)
        cv2.rectangle(panel, p_min, p_max, fov_color, 2, cv2.LINE_AA)

        # 3. Draw Camera Position & Bore-Sight Axis
        cam_p = self.world_to_panel_coords(state.camera_world_pos[0], state.camera_world_pos[1], world_w, world_h)
        cv2.drawMarker(panel, cam_p, (255, 200, 50), cv2.MARKER_DIAMOND, 14, 2, cv2.LINE_AA)
        
        # Line from camera position to FOV center
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
        b_color = (0, 255, 120) if state.is_visible else (0, 100, 255)  # Bright Green if visible, Red/Orange if outside FOV
        cv2.circle(panel, beacon_p, 6, b_color, -1, cv2.LINE_AA)
        cv2.circle(panel, beacon_p, 10, b_color, 1, cv2.LINE_AA)

        # Velocity vector
        vx, vy = state.beacon_world_velocity
        vel_end = (
            int(round(beacon_p[0] + vx * 0.2)),
            int(round(beacon_p[1] + vy * 0.2)),
        )
        cv2.arrowedLine(panel, beacon_p, vel_end, (0, 255, 255), 1, cv2.LINE_AA, tipLength=0.3)

        # Panel Header
        cv2.rectangle(panel, (0, 0), (self.world_panel_size, 26), (15, 18, 22), -1)
        cv2.putText(panel, f"WORLD VIEW [2000x2000 px]", (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Cam @ ({int(state.camera_world_pos[0])},{int(state.camera_world_pos[1])})", (self.world_panel_size - 170, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1, cv2.LINE_AA)

        return panel

    def render_camera_view(self, raw_frame: np.ndarray, state: GroundTruthState) -> np.ndarray:
        """
        Overlay annotations on the actual 640x480 monochrome sensor frame.
        Shows optical center crosshair, FOV bounds, and beacon detection status.
        """
        # Convert 8-bit single-channel monochrome frame to 3-channel BGR for colored overlays
        view = cv2.cvtColor(raw_frame, cv2.COLOR_GRAY2BGR)

        # Optical Bore-Sight Crosshair at Principal Point (320, 240)
        cx, cy = 320, 240
        cross_color = (80, 120, 160)
        cv2.line(view, (cx - 20, cy), (cx + 20, cy), cross_color, 1, cv2.LINE_AA)
        cv2.line(view, (cx, cy - 20), (cx, cy + 20), cross_color, 1, cv2.LINE_AA)
        cv2.circle(view, (cx, cy), 15, cross_color, 1, cv2.LINE_AA)

        # Sensor frame border
        cv2.rectangle(view, (0, 0), (self.cam_width - 1, self.cam_height - 1), (80, 80, 80), 1)

        if state.is_visible and state.beacon_image_pos is not None:
            u, v = state.beacon_image_pos
            iu, iv = int(round(u)), int(round(v))

            # Draw target acquisition box
            box_sz = int(round(self.engine.beacon.size + 8))
            cv2.rectangle(
                view,
                (iu - box_sz // 2, iv - box_sz // 2),
                (iu + box_sz // 2, iv + box_sz // 2),
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

            # Bore-sight error vector (line from principal point to beacon centroid)
            cv2.line(view, (cx, cy), (iu, iv), (0, 255, 255), 1, cv2.LINE_AA)

            # Sub-pixel coordinate readout tag
            tag_text = f"({u:.1f}, {v:.1f})"
            cv2.putText(view, tag_text, (iu + 10, max(20, iv - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)
        else:
            # Out of FOV Alert Banner
            banner = np.zeros((40, self.cam_width, 3), dtype=np.uint8)
            banner[:] = (0, 0, 160)
            cv2.addWeighted(banner, 0.6, view[cy - 20:cy + 20, :], 0.4, 0, view[cy - 20:cy + 20, :])
            cv2.putText(
                view,
                "** TARGET OUTSIDE CAMERA FOV **",
                (self.cam_width // 2 - 165, cy + 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # Header Bar
        cv2.rectangle(view, (0, 0), (self.cam_width, 26), (15, 18, 22), -1)
        cv2.putText(view, "CAMERA SENSOR VIEW [640x480 | 4.0deg x 3.0deg FOV]", (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1, cv2.LINE_AA)
        
        status_text = "IN-FOV" if state.is_visible else "OUT-OF-FOV"
        status_color = (0, 255, 100) if state.is_visible else (0, 80, 255)
        cv2.putText(view, f"STATUS: {status_text}", (self.cam_width - 155, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, status_color, 1, cv2.LINE_AA)

        return view

    def render_hud_dashboard(self, state: GroundTruthState) -> np.ndarray:
        """
        Draw live telemetry readout and keyboard controls help HUD.
        """
        hud = np.zeros((self.hud_height, self.total_width, 3), dtype=np.uint8)
        hud[:] = (16, 20, 24)  # Dark console slate

        cv2.rectangle(hud, (0, 0), (self.total_width - 1, self.hud_height - 1), (50, 60, 72), 1)

        # Title bar
        cv2.rectangle(hud, (0, 0), (self.total_width, 24), (24, 30, 38), -1)
        cv2.putText(
            hud,
            "REAL-TIME SIMULATION TELEMETRY & GROUND TRUTH [PHASE 1 DEBUG VIEWER]",
            (15, 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
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
            0.45,
            pause_color,
            1,
            cv2.LINE_AA,
        )

        # Column 1: Time & Kinematics
        col1_x = 20
        y = 48
        dy = 22
        cv2.putText(hud, f"Sim Time:     {state.timestamp:6.3f} s  (Frame #{state.frame_index})", (col1_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 210, 220), 1)
        cv2.putText(hud, f"Motion Model: {self.current_motion.upper()}", (col1_x, y + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 255), 1)
        cv2.putText(hud, f"Beacon World: ({state.beacon_world_pos[0]:6.1f}, {state.beacon_world_pos[1]:6.1f}) px", (col1_x, y + 2 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 210, 220), 1)
        cv2.putText(hud, f"Beacon Speed: ({state.beacon_world_velocity[0]:+5.1f}, {state.beacon_world_velocity[1]:+5.1f}) px/s", (col1_x, y + 3 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 210, 220), 1)
        cv2.putText(hud, f"Camera World: ({state.camera_world_pos[0]:.1f}, {state.camera_world_pos[1]:.1f}) px", (col1_x, y + 4 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 210, 220), 1)

        # Column 2: Optics & Pointing Error
        col2_x = 420
        cv2.putText(hud, f"Camera Pan/Tilt:   Pan: {state.camera_pan_deg:+.2f} deg | Tilt: {state.camera_tilt_deg:+.2f} deg", (col2_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 210, 220), 1)
        
        vis_str = "TRUE (Target in FOV)" if state.is_visible else "FALSE (Outside FOV)"
        vis_col = (0, 255, 120) if state.is_visible else (0, 80, 255)
        cv2.putText(hud, f"Target Visibility: {vis_str}", (col2_x, y + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, vis_col, 1)

        if state.beacon_image_pos is not None:
            u, v = state.beacon_image_pos
            du, dv = state.pixel_error if state.pixel_error else (0.0, 0.0)
            dist_px = math.hypot(du, dv)
            az_err, el_err = state.angular_error_deg if state.angular_error_deg else (0.0, 0.0)
            cv2.putText(hud, f"Projected Image:   u={u:5.1f} px, v={v:5.1f} px", (col2_x, y + 2 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 210, 220), 1)
            cv2.putText(hud, f"Pixel Error (du,dv): du={du:+5.1f}, dv={dv:+5.1f} px (|E| = {dist_px:5.1f} px)", (col2_x, y + 3 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
            cv2.putText(hud, f"Angular Error:     dAz={az_err:+.3f} deg, dEl={el_err:+.3f} deg", (col2_x, y + 4 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 210, 220), 1)
        else:
            cv2.putText(hud, "Projected Image:   [TARGET NOT VISIBLE]", (col2_x, y + 2 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 130, 140), 1)
            cv2.putText(hud, "Pixel Error (du,dv): N/A", (col2_x, y + 3 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 130, 140), 1)
            cv2.putText(hud, "Angular Error:     N/A", (col2_x, y + 4 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 130, 140), 1)

        # Column 3: Interactive Key Controls Guide
        col3_x = 840
        cv2.putText(hud, "KEYBOARD SHORTCUTS:", (col3_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 255), 1)
        cv2.putText(hud, "[SPACE] : Play / Pause", (col3_x, y + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 190, 200), 1)
        cv2.putText(hud, "[R]     : Reset Simulation", (col3_x, y + 2 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 190, 200), 1)
        cv2.putText(hud, "[1-4]   : 1=Line 2=Circle 3=Fig-8 4=Rand", (col3_x, y + 3 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 190, 200), 1)
        cv2.putText(hud, "[S / F] : Slower / Faster | [Q/ESC] Quit", (col3_x, y + 4 * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 190, 200), 1)

        return hud

    def compose_dashboard(self, world_panel: np.ndarray, camera_view: np.ndarray, hud_panel: np.ndarray) -> np.ndarray:
        """Combine World View, Camera View, and HUD into a single composite frame."""
        canvas = np.zeros((self.total_height, self.total_width, 3), dtype=np.uint8)
        canvas[:] = (12, 14, 18)  # Deep canvas background

        # Place World View at top-left
        wx = self.margin
        wy = self.margin
        canvas[wy:wy + self.world_panel_size, wx:wx + self.world_panel_size] = world_panel

        # Place Camera View at top-right (vertically centered with world panel)
        cx = wx + self.world_panel_size + self.margin
        cy = wy + (self.world_panel_size - self.cam_height) // 2
        canvas[cy:cy + self.cam_height, cx:cx + self.cam_width] = camera_view

        # Place HUD at bottom
        hy = wy + self.world_panel_size + self.margin
        canvas[hy:hy + self.hud_height, self.margin:self.margin + self.total_width - 2 * self.margin] = hud_panel[:, self.margin:self.total_width - self.margin]

        return canvas

    def step_and_render(self) -> Tuple[np.ndarray, GroundTruthState]:
        """Advance one simulation step and return composite UI frame and state."""
        if not self.is_paused:
            dt = (1.0 / self.target_fps) * self.speed_multiplier
            raw_frame, state = self.engine.step(dt=dt)
            self.beacon_trail.append(state.beacon_world_pos)
            if len(self.beacon_trail) > self.trail_length:
                self.beacon_trail.pop(0)
        else:
            state = self.engine.get_state()
            is_vis, img_pos, _, _, _ = self.engine.renderer.render, state.beacon_image_pos
            raw_frame = self.engine.renderer.render(
                self.engine.beacon, self.engine.camera, state.is_visible, state.beacon_image_pos
            )

        world_panel = self.render_world_view(state)
        camera_view = self.render_camera_view(raw_frame, state)
        hud_panel = self.render_hud_dashboard(state)
        dashboard = self.compose_dashboard(world_panel, camera_view, hud_panel)

        return dashboard, state

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
