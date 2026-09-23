from __future__ import annotations

import math

DJI_MAVIC_FOV: tuple[float, float] = (68.0, 40.0)

GIMBAL_CORRECTION: float = 90.0

METERS_PER_DEGREE_LAT: float = 111_111.0


def apply_gimbal_correction(raw_gimbal_angle: float) -> float:
    """Add the fixed +90-degree offset that converts the DJI gimbal reading
    (0 = forward, -90 = nadir) into the pitch value expected by
    :func:`pixel_to_gps` (90 = nadir)."""
    return raw_gimbal_angle + GIMBAL_CORRECTION


def pixel_to_gps(
    pixel: tuple[float, float],
    img_size: tuple[int, int],
    fov: tuple[float, float],
    drone_info: tuple[float, float, float, float, float],
) -> tuple[float, float]:
    """Convert a pixel coordinate to a GPS (lat, lon) pair.

    Ported verbatim from ``cvn/app/waldo/waldo_run.py`` (lines 76-104).

    Parameters
    ----------
    pixel:
        ``(x, y)`` in image coordinates (origin top-left).
    img_size:
        ``(width, height)`` of the image in pixels.
    fov:
        ``(horizontal_deg, vertical_deg)`` field-of-view of the camera.
    drone_info:
        ``(lat, lon, alt_m, bearing_deg, pitch_deg)`` where *pitch_deg*
        should already include the gimbal correction (see
        :func:`apply_gimbal_correction`).

    Returns
    -------
    tuple[float, float]
        ``(latitude, longitude)`` of the projected ground point.
    """
    lat, lon, alt, bearing, pitch = drone_info

    dx = (pixel[0] - img_size[0] / 2) / (img_size[0] / 2) * (fov[0] / 2)
    dy = (
        ((img_size[1] - pixel[1]) - img_size[1] / 2)
        / (img_size[1] / 2)
        * (fov[1] / 2)
    )

    dy += pitch

    dx = alt * math.tan(math.radians(dx))
    dy = alt * math.tan(math.radians(dy))

    cos_b = math.cos(math.radians(-bearing))
    sin_b = math.sin(math.radians(-bearing))
    dx, dy = dx * cos_b - dy * sin_b, dx * sin_b + dy * cos_b

    lat += dy / METERS_PER_DEGREE_LAT
    lon += dx / (METERS_PER_DEGREE_LAT * math.cos(math.radians(lat)))

    return lat, lon
