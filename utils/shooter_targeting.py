from subsystems.shooter.constants import DIST_M, HOOD_DEG, RPM, shooter_offset
from utils.field_constants import Hub
from wpimath.geometry import Pose2d, Translation2d
import numpy as np

def shooter_distance_to_hub_m(robot_pose: Pose2d) -> float:
    """
    Returns 2D distance in meters from the shooter exit point to the hub center.

    Args:
        robot_pose (Pose2d): The robot's pose on the field
    """
    hub2d: Translation2d = Translation2d(Hub.INNER_CENTER_POINT.x, Hub.INNER_CENTER_POINT.y)

    # Shooter origin in field 2D: robot translation + rotated offset
    shooter_origin_field: Translation2d = robot_pose.translation() + shooter_offset.rotateBy(robot_pose.rotation())

    return shooter_origin_field.distance(hub2d)

def shot_setpoints_from_distance(distance_m: float) -> tuple[float, float]:
    hood_deg = float(np.interp(distance_m, DIST_M, HOOD_DEG))
    rpm = float(np.interp(distance_m, DIST_M, RPM))
    return hood_deg, rpm

def shot_setpoints_from_pose(robot_pose: Pose2d) -> tuple[float, float]:
    distance_m = shooter_distance_to_hub_m(robot_pose)
    hood_deg, rpm = shot_setpoints_from_distance(distance_m)
    return hood_deg, rpm
