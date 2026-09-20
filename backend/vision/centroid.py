"""
Centroid estimation algorithms for optical beacon spots.
Provides sub-pixel accuracy using intensity-weighted spatial moments.
"""

from typing import Tuple, Optional
import numpy as np
import cv2


def calculate_intensity_weighted_centroid(
    patch: np.ndarray,
    origin_offset: Tuple[int, int] = (0, 0),
) -> Tuple[float, float]:
    """
    Compute sub-pixel centroid using intensity-weighted spatial moments:
    m00 = sum(I)
    m10 = sum(x * I), m01 = sum(y * I)
    u_c = m10 / m00 + x_offset
    v_c = m01 / m00 + y_offset

    Args:
        patch: 2D numpy array of pixel intensities (uint8 or float)
        origin_offset: (x_min, y_min) offset of the patch within the parent image

    Returns:
        (u_subpixel, v_subpixel)
    """
    if patch.size == 0:
        return float(origin_offset[0]), float(origin_offset[1])

    # Convert to float for numerical stability
    p_float = patch.astype(np.float64)
    total_mass = np.sum(p_float)

    if total_mass <= 1e-9:
        # Fallback to geometric center of patch if all zero
        h, w = patch.shape[:2]
        return float(origin_offset[0] + w / 2.0), float(origin_offset[1] + h / 2.0)

    # Generate grid coordinates
    h, w = patch.shape[:2]
    ys, xs = np.mgrid[0:h, 0:w]

    m10 = np.sum(xs * p_float)
    m01 = np.sum(ys * p_float)

    u_local = m10 / total_mass
    v_local = m01 / total_mass

    return float(origin_offset[0] + u_local), float(origin_offset[1] + v_local)


def calculate_geometric_centroid(contour: np.ndarray) -> Tuple[float, float]:
    """
    Compute geometric centroid using standard binary contour moments.
    """
    m = cv2.moments(contour)
    if m["m00"] > 1e-6:
        return float(m["m10"] / m["m00"]), float(m["m01"] / m["m00"])
    else:
        # Fallback to bounding box center
        x, y, w, h = cv2.boundingRect(contour)
        return float(x + w / 2.0), float(y + h / 2.0)


def extract_subpixel_centroid(
    frame: np.ndarray,
    bbox: Tuple[int, int, int, int],
    method: str = "intensity_weighted",
    padding: int = 2,
) -> Tuple[float, float]:
    """
    Extract high-precision subpixel centroid from the frame within a bounded ROI.

    Args:
        frame: Full sensor image (H, W) uint8
        bbox: (umin, vmin, umax, vmax) integer bounding box
        method: 'intensity_weighted' or 'geometric'
        padding: Pixel border padding around bbox

    Returns:
        (u_subpixel, v_subpixel)
    """
    h_frame, w_frame = frame.shape[:2]
    umin, vmin, umax, vmax = bbox

    # Apply padding and clamp to image limits
    c_umin = max(0, umin - padding)
    c_vmin = max(0, vmin - padding)
    c_umax = min(w_frame, umax + padding)
    c_vmax = min(h_frame, vmax + padding)

    if c_umax <= c_umin or c_vmax <= c_vmin:
        return float(umin + (umax - umin) / 2.0), float(vmin + (vmax - vmin) / 2.0)

    patch = frame[c_vmin:c_vmax, c_umin:c_umax]

    if method == "intensity_weighted":
        return calculate_intensity_weighted_centroid(patch, origin_offset=(c_umin, c_vmin))
    else:
        # Geometric center
        return float(umin + (umax - umin) / 2.0), float(vmin + (vmax - vmin) / 2.0)
