"""
Sensor frame renderer module.
Generates 2D monochrome camera sensor images (640x480 uint8 numpy arrays)
representing the optical view of the beacon.
"""

import math
import numpy as np
import cv2
from typing import Tuple, Optional
from .beacon import Beacon
from .camera import Camera


class FrameRenderer:
    """
    Simulates optical sensor rasterization for monochrome camera frames.
    """

    def __init__(self, resolution: Tuple[int, int] = (640, 480)) -> None:
        self.width = resolution[0]
        self.height = resolution[1]

    def render(
        self,
        beacon: Beacon,
        camera: Camera,
        is_visible: bool,
        image_pos: Optional[Tuple[float, float]],
    ) -> np.ndarray:
        """
        Generate a synthetic 8-bit monochrome camera frame.
        
        Returns:
            np.ndarray of shape (H, W) with dtype uint8.
        """
        frame = np.zeros((self.height, self.width), dtype=np.uint8)

        if not is_visible or image_pos is None:
            return frame

        u, v = image_pos
        intensity = int(max(0, min(255, beacon.intensity)))

        if beacon.shape == "square":
            half = beacon.size / 2.0
            x1 = int(round(u - half))
            y1 = int(round(v - half))
            x2 = int(round(u + half))
            y2 = int(round(v + half))

            # Clamp coordinates to image frame boundaries
            cx1 = max(0, x1)
            cy1 = max(0, y1)
            cx2 = min(self.width, x2)
            cy2 = min(self.height, y2)

            if cx2 > cx1 and cy2 > cy1:
                frame[cy1:cy2, cx1:cx2] = intensity

        elif beacon.shape == "circle":
            radius = int(round(beacon.size / 2.0))
            center = (int(round(u)), int(round(v)))
            cv2.circle(frame, center, radius, color=intensity, thickness=-1)

        elif beacon.shape in ("gaussian", "gaussian_spot"):
            # Sub-pixel 2D Gaussian beam optical spot
            sigma = max(0.5, beacon.size / 4.0)
            radius = int(math.ceil(3.0 * sigma))
            
            x0 = int(math.floor(u))
            y0 = int(math.floor(v))

            x_min = max(0, x0 - radius)
            x_max = min(self.width, x0 + radius + 1)
            y_min = max(0, y0 - radius)
            y_max = min(self.height, y0 + radius + 1)

            if x_max > x_min and y_max > y_min:
                ys, xs = np.mgrid[y_min:y_max, x_min:x_max]
                dist_sq = (xs - u) ** 2 + (ys - v) ** 2
                spot = beacon.intensity * np.exp(-dist_sq / (2.0 * sigma ** 2))
                frame[y_min:y_max, x_min:x_max] = np.clip(spot, 0, 255).astype(np.uint8)

        else:
            # Fallback default to square
            half = beacon.size / 2.0
            x1 = max(0, int(round(u - half)))
            y1 = max(0, int(round(v - half)))
            x2 = min(self.width, int(round(u + half)))
            y2 = min(self.height, int(round(v + half)))
            if x2 > x1 and y2 > y1:
                frame[y1:y2, x1:x2] = intensity

        return frame
