from wpimath.geometry import Pose2d, Pose3d, Translation2d, Translation3d
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