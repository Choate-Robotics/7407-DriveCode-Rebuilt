from wpimath.geometry import Pose2d, Pose3d, Translation2d, Translation3d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from utils import alliance_flip_util, field_constants
import math

from subsystems.shooter.constants import DIST_M, HOOD_DEG, RPS, PASS_DIST_M, PASS_HOOD_DEG, PASS_RPS, TOF_DIST_M, TOF, lead_constant, tof_convergence_threshold_m, tof_iterations
from utils.field_constants import Hub
import numpy as np

hub2d: Translation2d = alliance_flip_util.get_alliance(Translation2d(Hub.INNER_CENTER_POINT.x, Hub.INNER_CENTER_POINT.y))

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
    Computes shot (hood_deg, rps) from robot pose:
    - computes shooter exit point in field coords (robot translation + rotated shooter_offset)
    - measures distance to hub center
    - interpolates hood and rps from DIST_M tables
    """
    distance_m: float = shot_distance_from_pose(robot_pose)

    hood_deg: float = float(np.interp(distance_m, DIST_M, HOOD_DEG))
    rps: float = float(np.interp(distance_m, DIST_M, RPS))

    return hood_deg, rps

def shot_distance_from_pose(robot_pose: Pose2d, target=hub2d) -> float:
    return robot_pose.translation().distance(target)

#passing
def pass_setpoints_from_pose(robot_pose: Pose2d) -> tuple[float, float]:
    """
    Computes pass (hood_deg, rps) based on robot pose:
    - chooses the correct pass target (via get_pass_setpoint)
    - computes shooter-to-target distance using shooter_offset rotated by robot heading
    - interpolates hood and rps from PASS_* tables
    """
    pass_setpoint: Translation2d = alliance_flip_util.get_alliance(get_pass_setpoint(robot_pose))

    distance_m: float = robot_pose.translation().distance(pass_setpoint)

    hood_deg: float = float(np.interp(distance_m, PASS_DIST_M, PASS_HOOD_DEG))
    rps: float = float(np.interp(distance_m, PASS_DIST_M, PASS_RPS))

    return hood_deg, rps

def get_tof_from_distance(distance_m: float) -> float:
    """
    Computes time of flight based on distance:
    - interpolates time of flight from TOF tables
    """
    tof: float = float(np.interp(distance_m, TOF_DIST_M, TOF))

    return tof

def get_field_relative_velocity(robot_pose: Pose2d, speeds: ChassisSpeeds) -> Translation2d:
    """
    Convert robot-relative chassis speeds to field-relative translation velocity (vx, vy).
    """
    field = ChassisSpeeds.fromRobotRelativeSpeeds(
        speeds.vx, speeds.vy, speeds.omega, robot_pose.rotation()
    )
    return Translation2d(field.vx, field.vy)

def compute_virtual_target(robot_pose: Pose2d, speeds: ChassisSpeeds, target: Translation2d) -> Translation2d:
    """
    Computes a virtual target position that accounts for the robot's movement during the time of flight of the ball.
    - uses an iterative approach to account for the fact that the time of flight depends on the distance to the target, which changes as we shift the target
    """
    field_relative_velocity: Translation2d = get_field_relative_velocity(robot_pose, speeds)

    # initial guesses
    distance = robot_pose.translation().distance(target)

    # iterate
    for _ in range(tof_iterations):
        tof = get_tof_from_distance(distance)

        shift_x = field_relative_velocity.X() * tof * lead_constant
        shift_y = field_relative_velocity.Y() * tof * lead_constant

        virtual_target: Translation2d = Translation2d(target.X() - shift_x, target.Y() - shift_y)

        # update distance
        prev = distance
        distance = robot_pose.translation().distance(virtual_target)

        # check convergence        
        if abs(distance - prev) < tof_convergence_threshold_m:
            break

    return virtual_target