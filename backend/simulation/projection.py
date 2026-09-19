"""
Projection and coordinate transformation module.
Implements the rigorous mathematical mapping between:
1. World coordinates (Xw, Yw)
2. Camera pan/tilt angular offsets (Azimuth, Elevation)
3. Sensor image-plane pixel coordinates (u, v)
"""

from typing import Tuple, Optional
from .beacon import Beacon
from .camera import Camera


class Projector:
    """
    Mathematical projector mapping 2D world space and camera gimbal angles
    to camera sensor space.
    """

    @staticmethod
    def world_to_angular(
        beacon_world_pos: Tuple[float, float],
        camera_world_pos: Tuple[float, float],
        camera_pan_deg: float,
        camera_tilt_deg: float,
        scale_x_deg_px: float,
        scale_y_deg_px: float,
    ) -> Tuple[float, float]:
        """
        Compute the angular deviation of the target relative to the camera optical axis (bore-sight).
        
        Args:
            beacon_world_pos: (Xb, Yb) in world units.
            camera_world_pos: (Xcam, Ycam) in world units.
            camera_pan_deg: Current camera azimuth angle in degrees.
            camera_tilt_deg: Current camera elevation angle in degrees.
            scale_x_deg_px: Angular scale per horizontal pixel (deg/px).
            scale_y_deg_px: Angular scale per vertical pixel (deg/px).

        Returns:
            (azimuth_error_deg, elevation_error_deg) relative to camera bore-sight.
        """
        dx = beacon_world_pos[0] - camera_world_pos[0]
        dy = beacon_world_pos[1] - camera_world_pos[1]

        # Convert spatial delta to angular space, then subtract camera pointing angles
        az_error_deg = dx * scale_x_deg_px - camera_pan_deg
        el_error_deg = dy * scale_y_deg_px - camera_tilt_deg

        return az_error_deg, el_error_deg

    @staticmethod
    def angular_to_image(
        az_error_deg: float,
        el_error_deg: float,
        scale_x_deg_px: float,
        scale_y_deg_px: float,
        principal_point: Tuple[float, float],
    ) -> Tuple[float, float]:
        """
        Project angular bore-sight error onto sensor image plane (u, v).
        """
        u0, v0 = principal_point
        u = u0 + (az_error_deg / scale_x_deg_px)
        v = v0 + (el_error_deg / scale_y_deg_px)
        return u, v

    @staticmethod
    def image_to_angular(
        u: float,
        v: float,
        scale_x_deg_px: float,
        scale_y_deg_px: float,
        principal_point: Tuple[float, float],
    ) -> Tuple[float, float]:
        """
        Convert pixel coordinate displacement back to angular bore-sight offset.
        Inverse of angular_to_image.
        """
        u0, v0 = principal_point
        az_error_deg = (u - u0) * scale_x_deg_px
        el_error_deg = (v - v0) * scale_y_deg_px
        return az_error_deg, el_error_deg

    @classmethod
    def world_to_image(
        cls,
        beacon_world_pos: Tuple[float, float],
        camera: Camera,
    ) -> Tuple[float, float]:
        """
        Directly map world position to image pixel coordinates (u, v) given camera state.
        """
        az_err, el_err = cls.world_to_angular(
            beacon_world_pos=beacon_world_pos,
            camera_world_pos=camera.position,
            camera_pan_deg=camera.pan_deg,
            camera_tilt_deg=camera.tilt_deg,
            scale_x_deg_px=camera.scale_x_deg_px,
            scale_y_deg_px=camera.scale_y_deg_px,
        )
        return cls.angular_to_image(
            az_error_deg=az_err,
            el_error_deg=el_err,
            scale_x_deg_px=camera.scale_x_deg_px,
            scale_y_deg_px=camera.scale_y_deg_px,
            principal_point=camera.principal_point,
        )

    @staticmethod
    def is_box_in_fov(
        bbox: Tuple[float, float, float, float],
        width: int,
        height: int,
    ) -> bool:
        """
        Determine whether a bounding box (umin, vmin, umax, vmax) overlaps the camera FOV.
        """
        umin, vmin, umax, vmax = bbox
        return not (umax < 0 or umin >= width or vmax < 0 or vmin >= height)

    @classmethod
    def project_beacon(
        cls,
        beacon: Beacon,
        camera: Camera,
    ) -> Tuple[
        bool,
        Optional[Tuple[float, float]],
        Optional[Tuple[float, float, float, float]],
        Optional[Tuple[float, float]],
        Optional[Tuple[float, float]],
    ]:
        """
        Project beacon entity into camera view.
        
        Returns:
            is_visible: bool
            image_centroid: Optional[(u, v)]
            image_bbox: Optional[(umin, vmin, umax, vmax)]
            angular_error_deg: Optional[(az_err_deg, el_err_deg)]
            pixel_error: Optional[(u - u0, v - v0)]
        """
        az_err, el_err = cls.world_to_angular(
            beacon_world_pos=beacon.position,
            camera_world_pos=camera.position,
            camera_pan_deg=camera.pan_deg,
            camera_tilt_deg=camera.tilt_deg,
            scale_x_deg_px=camera.scale_x_deg_px,
            scale_y_deg_px=camera.scale_y_deg_px,
        )

        u, v = cls.angular_to_image(
            az_error_deg=az_err,
            el_error_deg=el_err,
            scale_x_deg_px=camera.scale_x_deg_px,
            scale_y_deg_px=camera.scale_y_deg_px,
            principal_point=camera.principal_point,
        )

        half_size = beacon.size / 2.0
        umin = u - half_size
        vmin = v - half_size
        umax = u + half_size
        vmax = v + half_size
        bbox = (umin, vmin, umax, vmax)

        is_visible = cls.is_box_in_fov(bbox, camera.width, camera.height)

        u0, v0 = camera.principal_point
        pixel_error = (u - u0, v - v0)
        angular_error = (az_err, el_err)

        if is_visible:
            return True, (u, v), bbox, angular_error, pixel_error
        else:
            return False, (u, v), bbox, angular_error, pixel_error
