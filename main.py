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
    Run a simulation demonstration for the specified number of steps with Phase 2 tracking.
    """
    from backend.config.vision_config import VisionConfig
    from backend.vision.pipeline import VisionPipeline
    from backend.evaluation.evaluator import TrackingEvaluator

    print("=" * 90)
    print(" SIH 2026: Problem Statement SIH26169 (ISRO / DOS)")
    print(" Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC")
    print(" Team: Greedy Minds | Phase 2: Classical CV Detection, Centroiding & Tracking")
    print("=" * 90)

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
    pipeline = VisionPipeline(VisionConfig())
    evaluator = TrackingEvaluator()

    print(f"[*] Running {steps} simulation steps with motion='{motion_type}'...")
    print("-" * 90)
    print(f"{'Step':<5} {'GT Image (u,v)':<16} {'Raw Det (u,v)':<16} {'Kalman (u,v)':<16} {'State':<12} {'Raw Err':<10} {'Filt Err'}")
    print("-" * 90)

    start_real_time = time.perf_counter()
    last_frame = None

    for i in range(steps):
        frame, gt_state = engine.step()
        last_frame = frame
        tracker_output = pipeline.process_frame(frame, dt=1.0 / fps)
        step_metrics = evaluator.evaluate_step(tracker_output, gt_state)

        # Print periodically (every 10 steps or first/last)
        if i % 10 == 0 or i == steps - 1:
            gt_pos_str = (
                f"({gt_state.beacon_image_pos[0]:.1f}, {gt_state.beacon_image_pos[1]:.1f})"
                if gt_state.beacon_image_pos is not None
                else "N/A"
            )
            raw_pos_str = (
                f"({tracker_output.raw_detection.centroid[0]:.1f}, {tracker_output.raw_detection.centroid[1]:.1f})"
                if tracker_output.raw_detection.is_detected and tracker_output.raw_detection.centroid is not None
                else "N/A"
            )
            kf_pos_str = (
                f"({tracker_output.filtered_centroid[0]:.1f}, {tracker_output.filtered_centroid[1]:.1f})"
                if tracker_output.filtered_centroid is not None
                else "N/A"
            )
            raw_err_val = step_metrics["raw_error_px"]
            filt_err_val = step_metrics["filtered_error_px"]
            raw_err_str = f"{raw_err_val:.2f} px" if raw_err_val is not None else "N/A"
            filt_err_str = f"{filt_err_val:.2f} px" if filt_err_val is not None else "N/A"

            print(
                f"{gt_state.frame_index:<5} {gt_pos_str:<16} {raw_pos_str:<16} "
                f"{kf_pos_str:<16} {tracker_output.state.value:<12} "
                f"{raw_err_str:<10} {filt_err_str}"
            )

    elapsed = time.perf_counter() - start_real_time
    sim_fps = steps / elapsed if elapsed > 0 else 0.0

    print("-" * 90)
    print(f"[+] Processed {steps} steps in {elapsed:.4f}s ({sim_fps:.1f} pipeline FPS)")
    print(f"[+] Sensor Frame Dimensions: {last_frame.shape[1]}x{last_frame.shape[0]} (Monochrome 8-bit)")

    # Print summary metrics from TrackingEvaluator
    summary = evaluator.get_summary_metrics()
    acq_str = f"{summary['acquisition_time_s']:.3f} s" if summary['acquisition_time_s'] is not None else "N/A"
    raw_mean_str = f"{summary['mean_raw_error_px']:.3f} px" if summary['mean_raw_error_px'] is not None else "N/A"
    raw_rmse_str = f"{summary['rmse_raw_error_px']:.3f} px" if summary['rmse_raw_error_px'] is not None else "N/A"
    filt_mean_str = f"{summary['mean_filtered_error_px']:.3f} px" if summary['mean_filtered_error_px'] is not None else "N/A"
    filt_rmse_str = f"{summary['rmse_filtered_error_px']:.3f} px" if summary['rmse_filtered_error_px'] is not None else "N/A"

    print("\n" + "=" * 45 + " TRACKING METRICS SUMMARY " + "=" * 45)
    print(f"Total Frames Processed:      {summary['total_frames']}")
    print(f"Frames Beacon Visible:       {summary['visible_frames']}")
    print(f"Frames Beacon Detected:      {summary['detected_frames']}")
    print(f"Detection Success Rate:      {summary['detection_rate_pct']:.1f}%")
    print(f"Target Loss Rate:            {summary['target_loss_rate_pct']:.1f}%")
    print(f"Acquisition Time:            {acq_str}")
    print(f"Mean Raw Centroid Error:     {raw_mean_str}")
    print(f"RMSE Raw Centroid Error:     {raw_rmse_str}")
    print(f"Mean Filtered Error:         {filt_mean_str}")
    print(f"RMSE Filtered Error:         {filt_rmse_str}")
    print(f"Mean Pipeline Execution Time:{summary['mean_processing_time_ms']:.2f} ms ({summary['processing_fps']:.1f} FPS)")
    print("=" * 90)

    if save_frame and last_frame is not None:
        import cv2
        filename = "simulation_sample_frame.png"
        cv2.imwrite(filename, last_frame)
        print(f"[+] Saved sample frame artifact: {filename}")


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
    parser.add_argument(
        "--view",
        "--gui",
        action="store_true",
        help="Launch the interactive real-time 30 Hz Phase 1 visualizer window",
    )

    args = parser.parse_args()

    if args.view:
        from visualization.viewer import launch_visualizer
        launch_visualizer(motion=args.motion, fps=args.fps)
    else:
        run_demo(
            motion_type=args.motion,
            steps=args.steps,
            fps=args.fps,
            shape=args.shape,
            save_frame=args.save_frame,
        )


if __name__ == "__main__":
    main()

