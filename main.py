"""
Main CLI entrypoint for SIH26169 FSOC Virtual Camera Tracking Simulation (Phase 1).
Runs a standalone simulation sanity demonstration and prints telemetry.
"""

import sys
import time
import argparse
from typing import Optional

from backend.config.simulation_config import (
    SimulationConfig,
    WorldConfig,
    BeaconConfig,
    CameraConfig,
)
from backend.engine import SimulationEngine


def run_demo(
    motion_type: str = "circular",
    steps: int = 90,
    fps: float = 30.0,
    shape: str = "square",
    save_frame: bool = False,
) -> None:
    """
    Run a simulation demonstration for the specified number of steps.
    """
    print("=" * 70)
    print(" SIH 2026: Problem Statement SIH26169 (ISRO / DOS)")
    print(" Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC")
    print(" Team: Greedy Minds | Phase 1: Foundational Simulation Engine")
    print("=" * 70)

    config = SimulationConfig(
        fps=fps,
        world=WorldConfig(width=2000.0, height=2000.0),
        beacon=BeaconConfig(
            initial_pos=(1000.0, 1000.0),
            size=10.0,
            shape=shape,
            intensity=255.0,
            motion_type=motion_type,
            motion_params={
                "radius": 150.0,
                "angular_speed_deg_s": 45.0,
                "amplitude_x": 200.0,
                "amplitude_y": 120.0,
                "period_s": 6.0,
                "speed": 50.0,
                "angle_deg": 30.0,
            },
        ),
        camera=CameraConfig(
            resolution=(640, 480),
            fov_deg=(4.0, 3.0),
            initial_pos=(1000.0, 1000.0),
            initial_pan_deg=0.0,
            initial_tilt_deg=0.0,
        ),
    )

    print(f"[*] Initializing Simulation Engine (FPS={config.fps}, World={config.world.width}x{config.world.height})...")
    engine = SimulationEngine(config)

    print(f"[*] Running {steps} simulation steps with motion='{motion_type}'...")
    print("-" * 70)
    print(f"{'Step':<6} {'SimTime':<9} {'Beacon World (X, Y)':<22} {'Visible':<8} {'Image (u, v)':<18} {'Pixel Err (du, dv)'}")
    print("-" * 70)

    start_real_time = time.perf_counter()
    last_frame = None

    for i in range(steps):
        frame, state = engine.step()
        last_frame = frame

        # Print periodically (every 10 steps or first/last)
        if i % 10 == 0 or i == steps - 1:
            beacon_pos_str = f"({state.beacon_world_pos[0]:.1f}, {state.beacon_world_pos[1]:.1f})"
            img_pos_str = (
                f"({state.beacon_image_pos[0]:.1f}, {state.beacon_image_pos[1]:.1f})"
                if state.beacon_image_pos is not None
                else "N/A"
            )
            pix_err_str = (
                f"({state.pixel_error[0]:.1f}, {state.pixel_error[1]:.1f})"
                if state.pixel_error is not None
                else "N/A"
            )
            print(
                f"{state.frame_index:<6} {state.timestamp:>6.3f}s  "
                f"{beacon_pos_str:<22} {str(state.is_visible):<8} "
                f"{img_pos_str:<18} {pix_err_str}"
            )

    elapsed = time.perf_counter() - start_real_time
    sim_fps = steps / elapsed if elapsed > 0 else 0.0

    print("-" * 70)
    print(f"[+] Completed {steps} steps in {elapsed:.4f}s ({sim_fps:.1f} simulated FPS)")
    print(f"[+] Sensor Frame Dimensions: {last_frame.shape[1]}x{last_frame.shape[0]} (Monochrome 8-bit)")

    if save_frame and last_frame is not None:
        import cv2
        filename = "simulation_sample_frame.png"
        cv2.imwrite(filename, last_frame)
        print(f"[+] Saved sample frame artifact: {filename}")

    print("=" * 70)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SIH26169 FSOC Virtual Camera Tracking Simulation Engine (Phase 1)"
    )
    parser.add_argument(
        "--motion",
        type=str,
        default="circular",
        choices=["straight_line", "circular", "figure_eight", "random"],
        help="Motion model for the beacon (default: circular)",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=60,
        help="Number of simulation steps to run (default: 60)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Simulation update frequency in Hz (default: 30.0)",
    )
    parser.add_argument(
        "--shape",
        type=str,
        default="square",
        choices=["square", "circle", "gaussian"],
        help="Beacon spot shape profile (default: square)",
    )
    parser.add_argument(
        "--save-frame",
        action="store_true",
        help="Save the final sensor frame as a PNG file",
    )

    args = parser.parse_args()
    run_demo(
        motion_type=args.motion,
        steps=args.steps,
        fps=args.fps,
        shape=args.shape,
        save_frame=args.save_frame,
    )


if __name__ == "__main__":
    main()
