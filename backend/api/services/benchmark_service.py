"""
Benchmark Service Layer.
Executes standard and custom tracking benchmark scenarios asynchronously.
Publishes progress updates and comprehensive benchmark matrices.
"""

import asyncio
import logging
from typing import List, Dict, Any, Optional
from concurrent.futures import ThreadPoolExecutor

from ...evaluation.benchmark import TrackingBenchmarkRunner, BenchmarkScenarioResult
from ...config.vision_config import VisionConfig
from ..websocket_manager import ws_manager
from ..schemas import BenchmarkScenarioRequest

logger = logging.getLogger("api.benchmark_service")


class BenchmarkService:
    """
    Manages benchmark jobs in background threads and caches the latest results.
    """

    def __init__(self) -> None:
        self.executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="benchmark_worker")
        self.latest_results: List[Dict[str, Any]] = []
        self.is_running: bool = False
        self._lock = asyncio.Lock()

    def get_available_scenarios(self) -> List[Dict[str, Any]]:
        """Return list of predefined benchmark scenario templates."""
        return [
            {"id": "A1_static_clean", "name": "Static Beacon (Clean Gaussian)", "motion": "static", "shape": "gaussian", "noise": 0.0},
            {"id": "A2_static_square", "name": "Static Beacon (Clean Square 10x10)", "motion": "static", "shape": "square", "noise": 0.0},
            {"id": "B1_linear_clean", "name": "Linear Motion (50 px/s, Clean)", "motion": "straight_line", "shape": "gaussian", "noise": 0.0},
            {"id": "B2_linear_square", "name": "Linear Motion (50 px/s, Square)", "motion": "straight_line", "shape": "square", "noise": 0.0},
            {"id": "C1_circular_clean", "name": "Circular Motion (R=150px, w=45 deg/s, Clean)", "motion": "circular", "shape": "gaussian", "noise": 0.0},
            {"id": "C2_circular_square", "name": "Circular Motion (R=150px, w=45 deg/s, Square)", "motion": "circular", "shape": "square", "noise": 0.0},
            {"id": "D1_figure8_clean", "name": "Figure-8 Lemniscate (Clean)", "motion": "figure_eight", "shape": "gaussian", "noise": 0.0},
            {"id": "E1_fast_linear", "name": "High-Speed Linear (120 px/s, Clean)", "motion": "straight_line", "shape": "gaussian", "noise": 0.0},
            {"id": "F1_noisy_0.5px", "name": "Noisy Linear (sigma=0.5 px)", "motion": "straight_line", "shape": "gaussian", "noise": 0.5},
            {"id": "F2_noisy_1.0px", "name": "Noisy Linear (sigma=1.0 px)", "motion": "straight_line", "shape": "gaussian", "noise": 1.0},
            {"id": "F3_noisy_2.0px", "name": "Noisy Linear (sigma=2.0 px)", "motion": "straight_line", "shape": "gaussian", "noise": 2.0},
            {"id": "F4_noisy_5.0px", "name": "Noisy Linear (sigma=5.0 px)", "motion": "straight_line", "shape": "gaussian", "noise": 5.0},
            {"id": "G1_dropout_linear", "name": "Temporary Loss (5 frames dropout on Linear)", "motion": "straight_line", "shape": "gaussian", "dropout": [40, 44]},
            {"id": "G2_dropout_circular", "name": "Temporary Loss (5 frames dropout on Circular)", "motion": "circular", "shape": "gaussian", "dropout": [40, 44]},
        ]

    async def run_full_matrix(self, vision_config: Optional[VisionConfig] = None) -> List[Dict[str, Any]]:
        """Run complete 14-scenario benchmark matrix in thread pool."""
        async with self._lock:
            self.is_running = True
            await ws_manager.broadcast_event(
                category="BENCHMARK",
                message="Starting full 14-scenario benchmark execution...",
                level="INFO",
            )

            loop = asyncio.get_event_loop()
            runner = TrackingBenchmarkRunner(fps=30.0, steps=120)

            def _execute_sync():
                return runner.run_full_benchmark_matrix(vision_config=vision_config)

            results: List[BenchmarkScenarioResult] = await loop.run_in_executor(self.executor, _execute_sync)

            formatted_results = [self._format_result(r) for r in results]
            self.latest_results = formatted_results
            self.is_running = False

            await ws_manager.broadcast_event(
                category="BENCHMARK",
                message="Benchmark matrix execution completed successfully.",
                level="INFO",
            )
            await ws_manager.broadcast_json({
                "type": "benchmark_result",
                "results": formatted_results,
            })

            return formatted_results

    async def run_custom_scenario(self, req: BenchmarkScenarioRequest, vision_config: Optional[VisionConfig] = None) -> Dict[str, Any]:
        """Run a single custom benchmark scenario."""
        async with self._lock:
            self.is_running = True
            loop = asyncio.get_event_loop()
            runner = TrackingBenchmarkRunner(fps=req.fps, steps=req.steps)

            dropout = (req.dropout_start_frame, req.dropout_end_frame) if req.dropout_start_frame > 0 else None

            def _execute_single():
                return runner.run_scenario(
                    name=req.scenario_id or f"Custom ({req.motion_type})",
                    motion_type=req.motion_type,
                    shape=req.shape,
                    motion_params=req.motion_params,
                    vision_config=vision_config,
                    noise_std_px=req.noise_std_px,
                    dropout_interval=dropout,
                    notes="Custom user-configured benchmark run",
                )

            res: BenchmarkScenarioResult = await loop.run_in_executor(self.executor, _execute_single)
            formatted = self._format_result(res)
            self.latest_results = [formatted]
            self.is_running = False

            await ws_manager.broadcast_json({
                "type": "benchmark_result",
                "results": [formatted],
            })
            return formatted

    def _format_result(self, r: BenchmarkScenarioResult) -> Dict[str, Any]:
        """Convert BenchmarkScenarioResult dataclass to JSON dict."""
        return {
            "scenario_name": r.scenario_name,
            "motion_type": r.motion_type,
            "shape": r.shape,
            "noise_std_px": r.noise_std_px,
            "dropout_frames": r.dropout_frames,
            "total_frames": r.total_frames,
            "raw_mean_err_px": round(r.raw_mean_err_px, 3),
            "raw_rmse_px": round(r.raw_rmse_px, 3),
            "raw_max_err_px": round(r.raw_max_err_px, 2),
            "filtered_mean_err_px": round(r.filtered_mean_err_px, 3),
            "filtered_rmse_px": round(r.filtered_rmse_px, 3),
            "filtered_max_err_px": round(r.filtered_max_err_px, 2),
            "velocity_rmse_px_s": round(r.velocity_rmse_px_s, 2) if r.velocity_rmse_px_s is not None else None,
            "acquisition_time_s": round(r.acquisition_time_s, 3) if r.acquisition_time_s is not None else None,
            "coasting_max_err_px": round(r.coasting_max_err_px, 2) if r.coasting_max_err_px is not None else None,
            "reacquisition_latency_frames": r.reacquisition_latency_frames,
            "notes": r.notes,
        }


# Global singleton instance
benchmark_service = BenchmarkService()
