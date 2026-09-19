"""
Tests for GroundTruthState model and telemetry integrity.
"""

from backend.models.state import GroundTruthState


def test_ground_truth_fields():
    """Verify all mandatory ground-truth state fields are populated and typed properly."""
    state = GroundTruthState(
        timestamp=1.5,
        frame_index=45,
        beacon_world_pos=(1050.0, 980.0),
        beacon_world_velocity=(10.0, -5.0),
        camera_world_pos=(1000.0, 1000.0),
        camera_pan_deg=0.3125,
        camera_tilt_deg=-0.125,
        is_visible=True,
        beacon_image_pos=(320.0, 240.0),
        beacon_bounding_box=(315.0, 235.0, 325.0, 245.0),
        angular_error_deg=(0.0, 0.0),
        pixel_error=(0.0, 0.0),
    )

    assert state.timestamp == 1.5
    assert state.frame_index == 45
    assert state.beacon_world_pos == (1050.0, 980.0)
    assert state.beacon_world_velocity == (10.0, -5.0)
    assert state.is_visible is True
    assert state.beacon_image_pos == (320.0, 240.0)

    # Verify serialization
    data = state.to_dict()
    assert isinstance(data, dict)
    assert data["timestamp"] == 1.5
    assert data["frame_index"] == 45
    assert data["is_visible"] is True
