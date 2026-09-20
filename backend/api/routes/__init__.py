"""
API routes package.
"""

from .simulation_routes import router as simulation_router
from .config_routes import router as config_router
from .tracking_routes import router as tracking_router
from .benchmark_routes import router as benchmark_router

__all__ = [
    "simulation_router",
    "config_router",
    "tracking_router",
    "benchmark_router",
]
