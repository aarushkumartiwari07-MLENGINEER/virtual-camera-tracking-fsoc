"""
CLI Runner for the Phase 2 Kalman Tracking Benchmark Audit.
Executes the comprehensive benchmark matrix and outputs the engineering comparison table.
"""

from backend.evaluation.benchmark import TrackingBenchmarkRunner
from backend.config.vision_config import VisionConfig


def main() -> None:
    print("=" * 145)
    print(" SIH26169 Phase 2 Kalman Filter Benchmark Audit & Tracking Verification Matrix")
    print("=" * 145)

    runner = TrackingBenchmarkRunner(fps=30.0, steps=120, seed=42)
    results = runner.run_full_benchmark_matrix()

    headers = [
        ("Scenario", 48),
        ("Raw RMSE", 11),
        ("KF RMSE", 11),
        ("Raw Max", 10),
        ("KF Max", 10),
        ("Vel RMSE", 12),
        ("Coast Max", 11),
        ("Notes", 32),
    ]

    header_line = "| " + " | ".join(f"{name:<{w}}" for name, w in headers) + " |"
    sep_line = "|-" + "-|-".join("-" * w for _, w in headers) + "-|"

    print(header_line)
    print(sep_line)

    for r in results:
        vel_str = f"{r.velocity_rmse_px_s:.2f} px/s" if r.velocity_rmse_px_s is not None else "N/A"
        coast_str = f"{r.coasting_max_err_px:.2f} px" if r.coasting_max_err_px is not None else "N/A"
        
        row = [
            f"{r.scenario_name:<48}",
            f"{r.raw_rmse_px:>8.3f} px",
            f"{r.filtered_rmse_px:>8.3f} px",
            f"{r.raw_max_err_px:>7.2f} px",
            f"{r.filtered_max_err_px:>7.2f} px",
            f"{vel_str:>12}",
            f"{coast_str:>11}",
            f"{r.notes:<32}",
        ]
        print("| " + " | ".join(row) + " |")

    print("=" * 145)

    print("\n" + "=" * 105)
    print(" KALMAN TUNING TRADE-OFF STUDY: Process Noise (Q) vs Measurement Noise (R)")
    print("=" * 105)

    tuning_configs = [
        ("Conservative (q=15, r=1.0) [Baseline]", 15.0, 1.0),
        ("Moderate (q=40, r=0.5)", 40.0, 0.5),
        ("Balanced (q=80, r=0.2)", 80.0, 0.2),
        ("Responsive (q=150, r=0.1)", 150.0, 0.1),
        ("High-Bandwidth (q=300, r=0.05)", 300.0, 0.05),
    ]

    t_headers = [
        ("Tuning Profile", 38),
        ("Linear Clean", 14),
        ("Linear (1px Noise)", 20),
        ("Circular (150px)", 18),
        ("Figure-8", 12),
    ]
    print("| " + " | ".join(f"{name:<{w}}" for name, w in t_headers) + " |")
    print("|-" + "-|-".join("-" * w for _, w in t_headers) + "-|")

    for label, q_val, r_val in tuning_configs:
        v_cfg = VisionConfig()
        v_cfg.kalman.process_noise_std = q_val
        v_cfg.kalman.measurement_noise_std = r_val

        r_lin = runner.run_scenario("Lin", "straight_line", "gaussian", {"speed": 50.0, "angle_deg": 30.0}, vision_config=v_cfg)
        r_noisy = runner.run_scenario("Noisy", "straight_line", "gaussian", {"speed": 50.0, "angle_deg": 30.0}, noise_std_px=1.0, vision_config=v_cfg)
        r_circ = runner.run_scenario("Circ", "circular", "gaussian", {"radius": 150.0, "angular_speed_deg_s": 45.0}, vision_config=v_cfg)
        r_fig = runner.run_scenario("Fig8", "figure_eight", "gaussian", {"amplitude_x": 200.0, "amplitude_y": 120.0, "period_s": 6.0}, vision_config=v_cfg)

        t_row = [
            f"{label:<38}",
            f"{r_lin.filtered_rmse_px:>11.3f} px",
            f"{r_noisy.filtered_rmse_px:>17.3f} px",
            f"{r_circ.filtered_rmse_px:>15.3f} px",
            f"{r_fig.filtered_rmse_px:>9.3f} px",
        ]
        print("| " + " | ".join(t_row) + " |")

    print("=" * 105)


if __name__ == "__main__":
    main()
