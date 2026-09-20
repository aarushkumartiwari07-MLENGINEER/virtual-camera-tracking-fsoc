"""
Benchmarking REST API routes.
"""

from typing import List
from fastapi import APIRouter, HTTPException
from ..schemas import BenchmarkScenarioRequest, BenchmarkResultResponse, CommandResponse
from ..services.benchmark_service import benchmark_service
from ..services.simulation_service import sim_service

router = APIRouter(prefix="/api/benchmarks", tags=["Benchmarks"])


@router.get("/scenarios")
async def get_available_scenarios():
    """Retrieve predefined benchmark scenario templates."""
    return benchmark_service.get_available_scenarios()


@router.get("/latest", response_model=List[BenchmarkResultResponse])
async def get_latest_benchmark_results():
    """Retrieve results of the most recent benchmark run."""
    return benchmark_service.latest_results


@router.post("/run")
async def run_benchmark(req: BenchmarkScenarioRequest):
    """Execute benchmark matrix or a single custom scenario."""
    if req.scenario_id == "all" or req.scenario_id is None:
        results = await benchmark_service.run_full_matrix(vision_config=sim_service.vision_config)
        return {
            "success": True,
            "message": f"Full benchmark matrix completed ({len(results)} scenarios).",
            "results": results,
        }
    else:
        result = await benchmark_service.run_custom_scenario(req, vision_config=sim_service.vision_config)
        return {
            "success": True,
            "message": f"Scenario '{result['scenario_name']}' completed.",
            "results": [result],
        }
