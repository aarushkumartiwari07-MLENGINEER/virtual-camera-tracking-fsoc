"""
World environment module for the FSOC virtual simulation.
Defines the spatial boundaries and boundary handling policies.
"""

from typing import Tuple


class World:
    """
    2D virtual space representing the scene/tracking plane for coarse alignment.
    Coordinates range from (0, 0) to (width, height).
    """

    def __init__(self, width: float = 2000.0, height: float = 2000.0, boundary_behavior: str = "bounce") -> None:
        if width <= 0 or height <= 0:
            raise ValueError(f"World dimensions must be positive, got width={width}, height={height}")
        self.width = float(width)
        self.height = float(height)
        self.boundary_behavior = boundary_behavior.lower()

    @property
    def center(self) -> Tuple[float, float]:
        """Return the geometric center (x, y) of the world."""
        return (self.width / 2.0, self.height / 2.0)

    @property
    def bounds(self) -> Tuple[float, float]:
        """Return (width, height)."""
        return (self.width, self.height)

    def is_inside(self, x: float, y: float) -> bool:
        """Check if a coordinate (x, y) lies inside the world boundaries."""
        return 0.0 <= x <= self.width and 0.0 <= y <= self.height

    def apply_boundary_conditions(
        self, x: float, y: float, vx: float, vy: float
    ) -> Tuple[float, float, float, float]:
        """
        Handle boundary interactions based on configured policy.
        Returns: (x_new, y_new, vx_new, vy_new)
        """
        if self.boundary_behavior == "bounce":
            if x < 0.0:
                x = -x
                vx = abs(vx)
            elif x > self.width:
                x = 2.0 * self.width - x
                vx = -abs(vx)

            if y < 0.0:
                y = -y
                vy = abs(vy)
            elif y > self.height:
                y = 2.0 * self.height - y
                vy = -abs(vy)

            # Clamp after reflection to handle large timesteps
            x = max(0.0, min(self.width, x))
            y = max(0.0, min(self.height, y))

        elif self.boundary_behavior == "wrap":
            x = x % self.width
            y = y % self.height

        elif self.boundary_behavior == "clamp":
            x = max(0.0, min(self.width, x))
            y = max(0.0, min(self.height, y))

        return x, y, vx, vy
