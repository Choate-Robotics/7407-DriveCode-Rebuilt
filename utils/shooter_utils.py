from wpimath.geometry import Pose2d, Pose3d, Translation2d, Translation3d
from utils import alliance_flip_util, field_constants
import robot_constants
import math

def angle_aim_to_target(robot_pose: Pose2d, target: Pose2d | Pose3d | Translation2d | Translation3d) -> float:
    """
    Calculates the angle the robot needs to face to aim at a target position.

    :param robot_pose: The current pose of the robot.
    :param target: The target position to aim at.
    :return: The angle in radians the robot needs to face.
    """
    delta_x = target.X() - robot_pose.X()
    delta_y = target.Y() - robot_pose.Y()
    return math.atan2(delta_y, delta_x)

def get_pass_setpoint(robot_pose: Pose2d) -> Translation2d:
    """
    Calculates the pass setpoint based on the robot's current position.

    :param robot_pose: The current pose of the robot.
    :return: The calculated pass setpoint as a Translation2d.
    """

    if alliance_flip_util.get_y(robot_pose.Y()) < (field_constants.LinesHorizontal.CENTER):
        # from lowest quadrant
        if alliance_flip_util.get_y(robot_pose.Y()) < (field_constants.LinesHorizontal.CENTER - robot_constants.pass_offset):
            return alliance_flip_util.get_alliance(robot_constants.pass_target_2)
        # from middle-lower quadrant
        else:
            return alliance_flip_util.get_alliance(robot_constants.pass_target_1)
    else:
        # from upper quadrant
        if alliance_flip_util.get_y(robot_pose.Y()) > (field_constants.LinesHorizontal.CENTER + robot_constants.pass_offset):
            return alliance_flip_util.get_alliance(field_constants.LinesHorizontal.CENTER + robot_constants.pass_target_2)
        # from middle-upper quadrant
        else:
            return alliance_flip_util.get_alliance(field_constants.LinesHorizontal.CENTER + robot_constants.pass_target_1)