"""
Simulation components for the FSOC Virtual Camera tracking system.
"""

from .world import World
from .beacon import Beacon
from .camera import Camera
from .projection import Projector
from .renderer import FrameRenderer
from .motion import (
    MotionModel,
    StraightLineMotion,
    CircularMotion,
    FigureEightMotion,
    RandomMotion,
    create_motion_model,
)

__all__ = [
    "World",
    "Beacon",
    "Camera",
    "Projector",
    "FrameRenderer",
    "MotionModel",
    "StraightLineMotion",
    "CircularMotion",
    "FigureEightMotion",
    "RandomMotion",
    "create_motion_model",
]
