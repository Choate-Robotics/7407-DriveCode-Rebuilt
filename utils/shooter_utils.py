from wpimath.geometry import Pose2d, Pose3d, Translation2d, Translation3d, Rotation2d
from utils import alliance_flip_util, field_constants
import math

from subsystems.shooter.constants import DIST_M, HOOD_DEG, RPM, shooter_offset, PASS_DIST_M, PASS_HOOD_DEG, PASS_RPM
from utils.field_constants import Hub
import numpy as np

def angle_aim_to_target(robot_pose: Pose2d, target: Pose2d | Pose3d | Translation2d | Translation3d) -> Rotation2d:
    """
    Calculates the angle the robot needs to face to aim at a target position.

    :param robot_pose: The current pose of the robot.
    :param target: The target position to aim at.
    :return: The Rotation2d the robot needs to face.
    """
    delta_x = target.X() - robot_pose.X()
    delta_y = target.Y() - robot_pose.Y()
    return Rotation2d.fromDegrees(math.degrees(math.atan2(delta_y, delta_x)))

def get_pass_setpoint(robot_pose: Pose2d) -> Translation2d:
    """
    Calculates the pass setpoint based on the robot's current position.

    :param robot_pose: The current pose of the robot.
    :return: The calculated pass setpoint as a Translation2d.
    """

    if alliance_flip_util.get_y(robot_pose.Y()) < (field_constants.LinesHorizontal.CENTER):
        # from lowest quadrant
        if alliance_flip_util.get_y(robot_pose.Y()) < (field_constants.LinesHorizontal.CENTER - field_constants.pass_offset):
            return alliance_flip_util.get_alliance(field_constants.pass_target_2)
        # from middle-lower quadrant
        else:
            return alliance_flip_util.get_alliance(field_constants.pass_target_1)
    else:
        # from upper quadrant
        if alliance_flip_util.get_y(robot_pose.Y()) > (field_constants.LinesHorizontal.CENTER + field_constants.pass_offset):
            return alliance_flip_util.get_alliance(Translation2d(field_constants.pass_target_2.X(), field_constants.LinesHorizontal.CENTER + field_constants.pass_target_2.Y()))
        # from middle-upper quadrant
        else:
            return alliance_flip_util.get_alliance(Translation2d(field_constants.pass_target_1.X(), field_constants.LinesHorizontal.CENTER + field_constants.pass_target_1.Y()))
        
#shooting        
def shot_setpoints_from_pose(robot_pose: Pose2d) -> tuple[float, float]:
    """
    Computes shot (hood_deg, rpm) from robot pose:
    - computes shooter exit point in field coords (robot translation + rotated shooter_offset)
    - measures distance to hub center
    - interpolates hood and rpm from DIST_M tables
    """
    hub2d: Translation2d = Translation2d(Hub.INNER_CENTER_POINT.x, Hub.INNER_CENTER_POINT.y)

    shooter_origin_field: Translation2d = (
        robot_pose.translation() + shooter_offset.rotateBy(robot_pose.rotation())
    )

    distance_m: float = shooter_origin_field.distance(hub2d)

    hood_deg: float = float(np.interp(distance_m, DIST_M, HOOD_DEG))
    rpm: float = float(np.interp(distance_m, DIST_M, RPM))

    return hood_deg, rpm

#passing
def pass_setpoints_from_pose(robot_pose: Pose2d) -> tuple[float, float]:
    """
    Computes pass (hood_deg, rpm) based on robot pose:
    - chooses the correct pass target (via get_pass_setpoint)
    - computes shooter-to-target distance using shooter_offset rotated by robot heading
    - interpolates hood and rpm from PASS_* tables
    """
    pass_setpoint: Translation2d = get_pass_setpoint(robot_pose)

    shooter_origin_field: Translation2d = (
        robot_pose.translation() + shooter_offset.rotateBy(robot_pose.rotation())
    )

    distance_m: float = shooter_origin_field.distance(pass_setpoint)

    hood_deg: float = float(np.interp(distance_m, PASS_DIST_M, PASS_HOOD_DEG))
    rpm: float = float(np.interp(distance_m, PASS_DIST_M, PASS_RPM))

    return hood_deg, rpm