"""
Tests for BeaconDetector.
Validates:
1. Clean default beacon detection
2. Beacon at center, off-center, corner, and near boundary
3. Different beacon sizes (5x5, 10x10, 15x15, 20x20)
4. Different supported beacon shapes (Square, Circle, Gaussian)
5. Empty / black frame handling
6. Multiple bright candidate extraction and deterministic scoring
7. Distractor rejection and parameter configuration
"""

import pytest
import numpy as np
import cv2

from backend.vision.detector import BeaconDetector
from backend.config.vision_config import DetectorConfig
from backend.simulation.renderer import FrameRenderer
from backend.simulation.beacon import Beacon
from backend.simulation.camera import Camera
from backend.config.simulation_config import BeaconConfig, CameraConfig


def render_test_spot(
    resolution=(640, 480),
    pos=(320.0, 240.0),
    size=10.0,
    shape="square",
    intensity=255.0,
) -> np.ndarray:
    """Helper to render a standalone test spot frame using FrameRenderer."""
    renderer = FrameRenderer(resolution=resolution)
    beacon = Beacon(BeaconConfig(initial_pos=(1000.0, 1000.0), size=size, shape=shape, intensity=intensity))
    camera = Camera(CameraConfig(resolution=resolution))
    return renderer.render(beacon=beacon, camera=camera, is_visible=True, image_pos=pos)


def test_detector_center_square():
    """Verify detection of a standard 10x10 square beacon at center (320, 240)."""
    frame = render_test_spot(pos=(320.0, 240.0), size=10.0, shape="square")
    detector = BeaconDetector()
    result = detector.detect(frame)

    assert result.is_detected is True
    assert result.centroid is not None
    assert pytest.approx(result.centroid[0], abs=0.5) == 320.0
    assert pytest.approx(result.centroid[1], abs=0.5) == 240.0
    assert result.selected_candidate is not None
    assert result.selected_candidate.peak_intensity == 255.0


def test_detector_off_center_gaussian():
    """Verify subpixel detection of a Gaussian spot at arbitrary off-center coordinate (415.7, 182.3)."""
    true_pos = (415.7, 182.3)
    frame = render_test_spot(pos=true_pos, size=12.0, shape="gaussian")
    detector = BeaconDetector()
    result = detector.detect(frame)

    assert result.is_detected is True
    assert pytest.approx(result.centroid[0], abs=0.3) == true_pos[0]
    assert pytest.approx(result.centroid[1], abs=0.3) == true_pos[1]


def test_detector_near_boundary():
    """Verify beacon detection when positioned near image boundary (15.0, 15.0)."""
    frame = render_test_spot(pos=(15.0, 15.0), size=10.0, shape="square")
    detector = BeaconDetector()
    result = detector.detect(frame)

    assert result.is_detected is True
    assert pytest.approx(result.centroid[0], abs=0.5) == 15.0
    assert pytest.approx(result.centroid[1], abs=0.5) == 15.0


@pytest.mark.parametrize("size", [5.0, 8.0, 10.0, 15.0, 20.0])
def test_detector_various_sizes(size):
    """Verify detection across the full PS-mandated 5px-20px target size range."""
    frame = render_test_spot(pos=(300.0, 200.0), size=size, shape="square")
    detector = BeaconDetector()
    result = detector.detect(frame)

    assert result.is_detected is True
    assert pytest.approx(result.centroid[0], abs=0.5) == 300.0
    assert pytest.approx(result.centroid[1], abs=0.5) == 200.0


@pytest.mark.parametrize("shape", ["square", "circle", "gaussian"])
def test_detector_all_shapes(shape):
    """Verify detection across all supported optical spot profiles."""
    frame = render_test_spot(pos=(250.0, 350.0), size=12.0, shape=shape)
    detector = BeaconDetector()
    result = detector.detect(frame)

    assert result.is_detected is True
    assert pytest.approx(result.centroid[0], abs=0.5) == 250.0
    assert pytest.approx(result.centroid[1], abs=0.5) == 350.0


def test_detector_empty_frame():
    """Verify detector correctly reports is_detected=False on black / empty frame."""
    frame = np.zeros((480, 640), dtype=np.uint8)
    detector = BeaconDetector()
    result = detector.detect(frame)

    assert result.is_detected is False
    assert result.selected_candidate is None
    assert len(result.all_candidates) == 0


def test_detector_multiple_candidates_selection():
    """Verify detector extracts multiple candidates and deterministically selects the brightest/primary beacon."""
    frame = np.zeros((480, 640), dtype=np.uint8)

    # Candidate 1: Dim distractor at (100, 100)
    frame[95:105, 95:105] = 80
    # Candidate 2: Bright target beacon at (400, 300)
    frame[295:305, 395:405] = 255

    detector = BeaconDetector()
    result = detector.detect(frame)

    assert result.is_detected is True
    assert len(result.all_candidates) == 2
    # The selected candidate should be the bright beacon near (400, 300)
    assert pytest.approx(result.centroid[0], abs=1.0) == 400.0
    assert pytest.approx(result.centroid[1], abs=1.0) == 300.0


def test_detector_predicted_prior_gating():
    """Verify detector prioritizes candidate closer to temporal prior when multiple similar blobs exist."""
    frame = np.zeros((480, 640), dtype=np.uint8)

    # Blob A at (200, 200) intensity 250
    frame[195:205, 195:205] = 250
    # Blob B at (450, 450) intensity 250
    frame[445:455, 445:455] = 250

    detector = BeaconDetector()
    # Provide prior near Blob B (440, 440)
    result = detector.detect(frame, predicted_pos=(440.0, 440.0))

    assert result.is_detected is True
    assert pytest.approx(result.centroid[0], abs=1.0) == 450.0
    assert pytest.approx(result.centroid[1], abs=1.0) == 450.0


def test_detector_configurable_threshold():
    """Verify changing intensity_threshold excludes dim noise below threshold."""
    frame = np.zeros((480, 640), dtype=np.uint8)
    frame[100:110, 100:110] = 50  # Dim spot of intensity 50

    # With threshold 30: detected
    det_low = BeaconDetector(DetectorConfig(intensity_threshold=30.0))
    assert det_low.detect(frame).is_detected is True

    # With threshold 80: rejected
    det_high = BeaconDetector(DetectorConfig(intensity_threshold=80.0))
    assert det_high.detect(frame).is_detected is False
