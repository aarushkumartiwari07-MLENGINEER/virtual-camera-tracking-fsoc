"""
Pydantic schemas and data models for FastAPI REST endpoints and WebSocket messages.
Defines the strict API contract between the frontend and backend service layer.
"""

from typing import Tuple, List, Optional, Dict, Any
from pydantic import BaseModel, Field


# ==========================================
# Configuration Schemas
# ==========================================

class WorldConfigModel(BaseModel):
    width: float = Field(2000.0, ge=500.0, le=10000.0, description="World width in pixels")
    height: float = Field(2000.0, ge=500.0, le=10000.0, description="World height in pixels")


class BeaconConfigModel(BaseModel):
    initial_pos: Tuple[float, float] = Field((1000.0, 1000.0), description="Initial beacon world position (x, y)")
    size: float = Field(10.0, ge=1.0, le=100.0, description="Beacon spot size in pixels")
    shape: str = Field("square", description="Spot shape: 'square', 'circle', 'gaussian'")
    intensity: float = Field(255.0, ge=1.0, le=255.0, description="Peak spot intensity")
    motion_type: str = Field("circular", description="Motion model: 'straight_line', 'circular', 'figure_eight', 'random'")
    motion_params: Dict[str, Any] = Field(
        default_factory=lambda: {
            "radius": 150.0,
            "angular_speed_deg_s": 45.0,
            "amplitude_x": 200.0,
            "amplitude_y": 120.0,
            "period_s": 6.0,
            "speed": 50.0,
            "angle_deg": 30.0,
        },
        description="Motion model trajectory parameters",
    )


class CameraConfigModel(BaseModel):
    resolution: Tuple[int, int] = Field((640, 480), description="Camera resolution (width, height)")
    fov_deg: Tuple[float, float] = Field((4.0, 3.0), description="Field of view (horizontal_deg, vertical_deg)")
    initial_pos: Tuple[float, float] = Field((1000.0, 1000.0), description="Camera world position (x, y)")
    initial_pan_deg: float = Field(0.0, description="Initial pan angle in degrees")
    initial_tilt_deg: float = Field(0.0, description="Initial tilt angle in degrees")
    max_pan_speed_deg_s: float = Field(10.0, ge=0.1, le=90.0, description="Max pan speed in deg/s")
    max_tilt_speed_deg_s: float = Field(10.0, ge=0.1, le=90.0, description="Max tilt speed in deg/s")


class SimulationConfigModel(BaseModel):
    fps: float = Field(30.0, ge=1.0, le=120.0, description="Simulation update frequency in Hz")
    world: WorldConfigModel = Field(default_factory=WorldConfigModel)
    beacon: BeaconConfigModel = Field(default_factory=BeaconConfigModel)
    camera: CameraConfigModel = Field(default_factory=CameraConfigModel)


class DetectorConfigModel(BaseModel):
    intensity_threshold: float = Field(30.0, ge=0.0, le=255.0, description="Intensity threshold")
    min_area: float = Field(2.0, ge=0.5, description="Min candidate blob area")
    max_area: float = Field(1500.0, ge=5.0, description="Max candidate blob area")
    min_aspect_ratio: float = Field(0.2, ge=0.05, description="Min aspect ratio")
    max_aspect_ratio: float = Field(5.0, le=20.0, description="Max aspect ratio")
    use_adaptive_threshold: bool = Field(False, description="Use adaptive thresholding")


class CentroidConfigModel(BaseModel):
    method: str = Field("intensity_weighted", description="Centroiding method: 'intensity_weighted', 'geometric'")
    crop_padding: int = Field(2, ge=0, le=10, description="Crop padding for moment extraction")


class KalmanConfigModel(BaseModel):
    process_noise_std: float = Field(80.0, ge=0.1, le=1000.0, description="Process noise std (px/s^2)")
    measurement_noise_std: float = Field(0.2, ge=0.01, le=50.0, description="Measurement noise std (px)")
    initial_error_std: float = Field(10.0, ge=0.1, description="Initial position error std (px)")
    initial_velocity_std: float = Field(50.0, ge=0.1, description="Initial velocity error std (px/s)")


class StateMachineConfigModel(BaseModel):
    consecutive_acquire_frames: int = Field(3, ge=1, le=20, description="Frames to promote ACQUIRED -> TRACKING")
    max_lost_coasting_frames: int = Field(5, ge=1, le=60, description="Max coasting frames before LOST")
    consecutive_reacquire_frames: int = Field(2, ge=1, le=20, description="Frames to transition REACQUIRED -> TRACKING")


class MasterConfigModel(BaseModel):
    simulation: SimulationConfigModel = Field(default_factory=SimulationConfigModel)
    detector: DetectorConfigModel = Field(default_factory=DetectorConfigModel)
    centroid: CentroidConfigModel = Field(default_factory=CentroidConfigModel)
    kalman: KalmanConfigModel = Field(default_factory=KalmanConfigModel)
    state_machine: StateMachineConfigModel = Field(default_factory=StateMachineConfigModel)


# ==========================================
# Status & Response Schemas
# ==========================================

class SimulationStatusResponse(BaseModel):
    status: str = Field(..., description="'STOPPED', 'RUNNING', 'PAUSED'")
    frame_index: int
    timestamp: float
    fps: float
    target_fps: float
    motion_type: str
    is_beacon_visible: bool
    tracking_state: str


class CommandResponse(BaseModel):
    success: bool
    message: str
    status: str


# ==========================================
# Real-Time Telemetry Schemas (WebSocket / REST)
# ==========================================

class GroundTruthTelemetry(BaseModel):
    beacon_world_pos: Tuple[float, float]
    beacon_world_vel: Tuple[float, float]
    beacon_image_pos: Optional[Tuple[float, float]]
    camera_world_pos: Tuple[float, float]
    camera_pan_deg: float
    camera_tilt_deg: float
    fov_deg: Tuple[float, float]
    is_visible: bool
    boresight_pixel_error: Optional[Tuple[float, float]]
    boresight_angular_error_deg: Optional[Tuple[float, float]]


class VisionTrackingTelemetry(BaseModel):
    tracking_state: str
    is_detected: bool
    candidate_count: int
    raw_centroid: Optional[Tuple[float, float]]
    raw_bbox: Optional[Tuple[int, int, int, int]]
    raw_area: Optional[float]
    raw_peak_intensity: Optional[float]
    filtered_centroid: Optional[Tuple[float, float]]
    filtered_velocity_px_s: Optional[Tuple[float, float]]
    is_predicted: bool
    consecutive_detections: int
    consecutive_losses: int
    processing_time_ms: float


class InstantaneousErrors(BaseModel):
    raw_error_px: Optional[float]
    filtered_error_px: Optional[float]


class TelemetryPacket(BaseModel):
    frame_index: int
    timestamp: float
    dt: float
    ground_truth: GroundTruthTelemetry
    tracking: VisionTrackingTelemetry
    errors: InstantaneousErrors
    sensor_frame_base64: Optional[str] = None  # JPEG/PNG base64 (omitted if disabled)


class MetricsSummaryResponse(BaseModel):
    total_frames: int
    visible_frames: int
    detected_frames: int
    detection_rate_pct: float
    target_loss_rate_pct: float
    false_positives: int
    mean_raw_error_px: float
    rmse_raw_error_px: float
    mean_filtered_error_px: float
    rmse_filtered_error_px: float
    acquisition_time_s: Optional[float]
    mean_reacquisition_time_s: Optional[float]
    mean_processing_time_ms: float
    processing_fps: float


# ==========================================
# Benchmark Schemas
# ==========================================

class BenchmarkScenarioRequest(BaseModel):
    scenario_id: Optional[str] = None
    motion_type: str = "circular"
    shape: str = "gaussian"
    noise_std_px: float = 0.0
    dropout_start_frame: int = -1
    dropout_end_frame: int = -1
    steps: int = 120
    fps: float = 30.0
    motion_params: Optional[Dict[str, Any]] = None


class BenchmarkResultResponse(BaseModel):
    scenario_name: str
    motion_type: str
    shape: str
    noise_std_px: float
    dropout_frames: int
    total_frames: int
    raw_mean_err_px: float
    raw_rmse_px: float
    raw_max_err_px: float
    filtered_mean_err_px: float
    filtered_rmse_px: float
    filtered_max_err_px: float
    velocity_rmse_px_s: Optional[float]
    acquisition_time_s: Optional[float]
    coasting_max_err_px: Optional[float]
    reacquisition_latency_frames: Optional[int]
    notes: str
