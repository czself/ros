"""Shared, ROS-independent HOME alignment limits and retry policy."""

import math


HOME_XY_TOLERANCE_M = 0.03
HOME_YAW_TOLERANCE_RAD = 0.04
HOME_REFINEMENT_RETRY_LIMIT = 1
HOME_REFINEMENT_TIMEOUT_S = 12.0


def home_navigation_tolerances(xy_tolerance, yaw_tolerance):
    """Tighten HOME tolerances to the parking acceptance contract."""
    xy_tolerance = float(xy_tolerance)
    yaw_tolerance = float(yaw_tolerance)
    if (not math.isfinite(xy_tolerance) or xy_tolerance <= 0.0 or
            not math.isfinite(yaw_tolerance) or yaw_tolerance <= 0.0):
        raise ValueError('HOME navigation tolerances must be finite and positive')
    return (min(xy_tolerance, HOME_XY_TOLERANCE_M),
            min(yaw_tolerance, HOME_YAW_TOLERANCE_RAD))


def should_retry_home_alignment(result, action_succeeded, attempts):
    """Retry only a successful action whose final pose misses HOME tolerance."""
    return (attempts < HOME_REFINEMENT_RETRY_LIMIT and action_succeeded and
            result == 'ARRIVAL_MISMATCH')


def amcl_sample_is_newer(request_stamp_s, pose_stamp_s):
    """Accept only an AMCL pose captured after the requested laser update."""
    request_stamp_s = float(request_stamp_s)
    pose_stamp_s = float(pose_stamp_s)
    return (math.isfinite(request_stamp_s) and math.isfinite(pose_stamp_s) and
            pose_stamp_s > request_stamp_s)
