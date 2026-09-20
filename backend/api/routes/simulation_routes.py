"""
Simulation Control and Lifecycle REST API routes.
"""

from typing import Optional
from fastapi import APIRouter, HTTPException, Query
from ..schemas import SimulationStatusResponse, CommandResponse
from ..services.simulation_service import sim_service

router = APIRouter(prefix="/api/simulation", tags=["Simulation"])


@router.get("/status", response_model=SimulationStatusResponse)
async def get_simulation_status():
    """Retrieve current simulation execution state and status."""
    return sim_service.get_status()


@router.post("/start", response_model=CommandResponse)
async def start_simulation():
    """Start or resume background simulation."""
    await sim_service.start()
    return CommandResponse(
        success=True,
        message="Simulation started successfully.",
        status=sim_service.status,
    )


@router.post("/pause", response_model=CommandResponse)
async def pause_simulation():
    """Pause background simulation loop."""
    await sim_service.pause()
    return CommandResponse(
        success=True,
        message="Simulation paused.",
        status=sim_service.status,
    )


@router.post("/resume", response_model=CommandResponse)
async def resume_simulation():
    """Resume paused simulation."""
    await sim_service.resume()
    return CommandResponse(
        success=True,
        message="Simulation resumed.",
        status=sim_service.status,
    )


@router.post("/stop", response_model=CommandResponse)
async def stop_simulation():
    """Stop simulation loop."""
    await sim_service.stop()
    return CommandResponse(
        success=True,
        message="Simulation stopped.",
        status=sim_service.status,
    )


@router.post("/reset", response_model=CommandResponse)
async def reset_simulation(motion: Optional[str] = Query(None, description="Optional new motion type")):
    """Reset simulation engine, clear tracking history and reset evaluators."""
    await sim_service.reset(motion_type=motion)
    return CommandResponse(
        success=True,
        message="Simulation reset successfully.",
        status=sim_service.status,
    )


@router.post("/step")
async def step_frame():
    """Advance simulation by a single discrete frame (for debugging)."""
    packet = await sim_service.step_frame()
    return {
        "success": True,
        "message": "Single frame executed.",
        "telemetry": packet,
    }
