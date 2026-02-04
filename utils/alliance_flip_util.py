from field_constants import *
from wpimath.geometry import Translation2d, Translation3d, Pose2d, Pose3d, Rotation2d
from wpilib import DriverStation
import math

### Credit to Team 6328 Mechanical Advantage

def should_flip():
    return DriverStation.getAlliance() == DriverStation.Alliance().kRed

def get_x(x: float) -> float:
    return FIELD_LENGTH - x if should_flip() else x

def get_y(y: float) -> float:
    return FIELD_WIDTH - y if should_flip() else y

def get_alliance(translation: Translation2d) -> Translation2d:
    return Translation2d(get_x(translation.X()), get_y(translation.Y())) if should_flip() else translation

def get_alliance(rotation: Rotation2d) -> Rotation2d:
    return rotation.rotateBy(math.pi) if should_flip() else rotation

def get_alliance(pose: Pose2d) -> Pose2d:
    return Pose2d(get_alliance(pose.translation()), get_alliance(pose.rotation())) if should_flip() else pose

def get_alliance(translation: Translation3d) -> Translation3d:
    return Translation3d(get_x(translation.X()), get_y(translation.Y()), translation.Z())

def get_alliance(pose: Pose3d) -> Pose3d:
    return Pose3d(get_alliance(pose.translation()), get_alliance(pose.rotation())) if should_flip() else pose