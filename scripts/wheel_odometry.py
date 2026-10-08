"""ROS-independent helpers for wheel encoder odometry."""

import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def validate_yaw_scale(yaw_scale):
    """Accept only finite, positive calibration scales in a conservative range."""
    yaw_scale = float(yaw_scale)
    if not math.isfinite(yaw_scale) or not 0.5 <= yaw_scale <= 1.5:
        raise ValueError('odometry yaw scale must be finite and between 0.5 and 1.5')
    return yaw_scale


def signed_velocity(previous, current, dt):
    """Return forward and yaw velocity from two (x, y, yaw) encoder poses."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError('encoder interval must be positive and finite')
    if not all(math.isfinite(value) for value in previous + current):
        raise ValueError('encoder pose must be finite')
    turn = normalize_angle(current[2] - previous[2])
    heading = previous[2] + 0.5 * turn
    forward = ((current[0] - previous[0]) * math.cos(heading)
               + (current[1] - previous[1]) * math.sin(heading)) / dt
    return forward, turn / dt


def integrate_encoder_pose(previous_raw, current_raw, previous_corrected,
                           yaw_scale):
    """Apply a yaw scale while preserving encoder-derived forward distance."""
    yaw_scale = validate_yaw_scale(yaw_scale)
    if not all(math.isfinite(value) for value in
               previous_raw + current_raw + previous_corrected):
        raise ValueError('encoder pose must be finite')
    if yaw_scale == 1.0:
        return tuple(current_raw)

    raw_turn = normalize_angle(current_raw[2] - previous_raw[2])
    raw_heading = previous_raw[2] + 0.5 * raw_turn
    dx = current_raw[0] - previous_raw[0]
    dy = current_raw[1] - previous_raw[1]
    forward_distance = dx * math.cos(raw_heading) + dy * math.sin(raw_heading)

    corrected_turn = yaw_scale * raw_turn
    corrected_heading = previous_corrected[2] + 0.5 * corrected_turn
    return (
        previous_corrected[0] + forward_distance * math.cos(corrected_heading),
        previous_corrected[1] + forward_distance * math.sin(corrected_heading),
        previous_corrected[2] + corrected_turn,
    )
