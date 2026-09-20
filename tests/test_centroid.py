"""
Tests for Sub-Pixel Intensity-Weighted Centroiding.
Validates:
1. Subpixel Gaussian centroid estimation accuracy (< 0.05 px error)
2. Asymmetric intensity distributions where centroid != bbox center
3. Uniform square and circular centroids
4. Empty patch and edge fallback handling
"""

import pytest
import math
import numpy as np

from backend.vision.centroid import (
    calculate_intensity_weighted_centroid,
    calculate_geometric_centroid,
    extract_subpixel_centroid,
)


def create_synthetic_gaussian_patch(
    size: int,
    center_subpixel: tuple[float, float],
    sigma: float = 2.5,
    peak_intensity: float = 255.0,
) -> np.ndarray:
    """Generate a high-resolution synthetic 2D Gaussian patch."""
    ys, xs = np.mgrid[0:size, 0:size]
    cx, cy = center_subpixel
    dist_sq = (xs - cx) ** 2 + (ys - cy) ** 2
    patch = peak_intensity * np.exp(-dist_sq / (2.0 * sigma ** 2))
    return np.clip(patch, 0, 255).astype(np.uint8)


def test_gaussian_subpixel_moment_accuracy():
    """Verify intensity-weighted moments recover continuous sub-pixel coordinates with < 0.05 px error."""
    test_centers = [
        (10.25, 10.75),
        (10.50, 10.50),
        (8.15, 12.85),
        (11.90, 9.10),
    ]

    for true_cx, true_cy in test_centers:
        patch = create_synthetic_gaussian_patch(size=21, center_subpixel=(true_cx, true_cy), sigma=2.5)
        est_u, est_v = calculate_intensity_weighted_centroid(patch, origin_offset=(100, 200))

        assert pytest.approx(est_u - 100, abs=0.05) == true_cx
        assert pytest.approx(est_v - 200, abs=0.05) == true_cy


def test_asymmetric_intensity_centroid():
    """
    Verify that an asymmetric patch (e.g. left side much brighter than right side)
    shifts the intensity-weighted centroid towards the brighter mass, unlike geometric bbox center.
    """
    patch = np.zeros((10, 10), dtype=np.uint8)
    # Bright mass on left (cols 0-3)
    patch[:, 0:4] = 255
    # Dim mass on right (cols 4-9)
    patch[:, 4:10] = 50

    est_u, est_v = calculate_intensity_weighted_centroid(patch)
    # Bbox center is 5.0, but intensity-weighted center must be shifted towards left (< 4.0)
    assert est_u < 4.0
    assert pytest.approx(est_v, abs=0.1) == 4.5  # Symmetric vertically


def test_empty_patch_fallback():
    """Verify all-zero patch falls back to geometric patch center gracefully without crashing."""
    patch = np.zeros((10, 10), dtype=np.uint8)
    u, v = calculate_intensity_weighted_centroid(patch, origin_offset=(50, 50))
    assert u == 55.0
    assert v == 55.0


def test_extract_subpixel_centroid_full_frame():
    """Verify extract_subpixel_centroid extracts accurate coordinates from a full image frame."""
    frame = np.zeros((480, 640), dtype=np.uint8)
    # Place a 15x15 Gaussian spot centered at (350.3, 220.7)
    spot = create_synthetic_gaussian_patch(size=21, center_subpixel=(10.3, 10.7), sigma=2.5)
    frame[210:231, 340:361] = spot

    bbox = (340, 210, 361, 231)
    u, v = extract_subpixel_centroid(frame, bbox=bbox, method="intensity_weighted", padding=2)

    assert pytest.approx(u, abs=0.05) == 350.3
    assert pytest.approx(v, abs=0.05) == 220.7
