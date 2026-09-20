"""
Configuration Management REST API routes.
"""

from fastapi import APIRouter, HTTPException
from ..schemas import MasterConfigModel, CommandResponse
from ..services.simulation_service import sim_service

router = APIRouter(prefix="/api/config", tags=["Configuration"])


@router.get("", response_model=MasterConfigModel)
async def get_configuration():
    """Retrieve full master simulation and vision configuration."""
    return sim_service.get_current_config()


@router.put("", response_model=CommandResponse)
async def update_configuration(config: MasterConfigModel):
    """Apply updated configuration and re-initialize simulation engine."""
    await sim_service.update_config(config)
    return CommandResponse(
        success=True,
        message="Configuration updated successfully.",
        status=sim_service.status,
    )


@router.post("/reset", response_model=CommandResponse)
async def reset_configuration():
    """Reset configuration back to default values."""
    default_config = MasterConfigModel()
    await sim_service.update_config(default_config)
    return CommandResponse(
        success=True,
        message="Configuration reset to defaults.",
        status=sim_service.status,
    )
