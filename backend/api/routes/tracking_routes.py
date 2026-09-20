"""
Tracking State and Objective Metrics REST API routes.
"""

from fastapi import APIRouter
from ..schemas import MetricsSummaryResponse, CommandResponse
from ..services.simulation_service import sim_service

router = APIRouter(prefix="/api", tags=["Tracking & Metrics"])


@router.get("/tracking/state")
async def get_tracking_state():
    """Retrieve instantaneous tracking state, raw detection, and Kalman estimate."""
    status = sim_service.get_status()
    packet = sim_service.last_telemetry_packet
    return {
        "status": status,
        "latest_telemetry": packet,
    }


@router.get("/metrics", response_model=MetricsSummaryResponse)
async def get_metrics_summary():
    """Retrieve aggregate performance and tracking error metrics from TrackingEvaluator."""
    return sim_service.get_metrics_summary()


@router.get("/metrics/history")
async def get_metrics_history():
    """Retrieve error and state history buffers for frontend time-series graphs."""
    return sim_service.get_metrics_history()


@router.post("/metrics/reset", response_model=CommandResponse)
async def reset_metrics():
    """Reset evaluation accumulators and graph history."""
    sim_service.evaluator.reset()
    sim_service.error_history.clear()
    sim_service.state_history.clear()
    return CommandResponse(
        success=True,
        message="Metrics and history reset.",
        status=sim_service.status,
    )
