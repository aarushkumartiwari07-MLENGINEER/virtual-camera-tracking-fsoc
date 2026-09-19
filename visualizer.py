"""
Convenience launcher for SIH26169 Phase 1 Simulation Visualizer.
Runs real-time side-by-side World View, Camera Sensor View, and Telemetry HUD.
"""

import argparse
from visualization.viewer import launch_visualizer


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SIH26169 FSOC Virtual Camera Tracking - Phase 1 Visualizer"
    )
    parser.add_argument(
        "--motion",
        type=str,
        default="circular",
        choices=["straight_line", "circular", "figure_eight", "random"],
        help="Initial motion model (default: circular). Can also switch at runtime with keys 1-4.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Target update frequency in Hz (default: 30.0)",
    )
    args = parser.parse_args()
    launch_visualizer(motion=args.motion, fps=args.fps)


if __name__ == "__main__":
    main()
