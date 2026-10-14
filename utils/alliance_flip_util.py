from utils.field_constants import *
from wpimath.geometry import Translation2d, Translation3d, Pose2d, Pose3d, Rotation2d
from wpilib import DriverStation
import math
from typing import overload

### Credit to Team 6328 Mechanical Advantage

# should_flip() is called many times per loop (aim commands, auto paths), so the
# alliance is read once per loop by refresh(), called at the start of robotPeriodic.
# Until the first refresh() (e.g. while autos are built in robotInit) it's read live.
_flip: bool | None = None
_ROTATE_180 = Rotation2d.fromDegrees(180)


def _read_flip() -> bool:
    alliance = DriverStation.getAlliance()
    return alliance == DriverStation.Alliance.kRed if alliance is not None else False


def refresh() -> None:
    global _flip
    _flip = _read_flip()


def should_flip() -> bool:
    return _flip if _flip is not None else _read_flip()

def get_x(x: float) -> float:
    return FIELD_LENGTH - x if should_flip() else x

def get_y(y: float) -> float:
    return FIELD_WIDTH - y if should_flip() else y

@overload
def get_alliance(x: Pose2d) -> Pose2d: ...
@overload
def get_alliance(x: Pose3d) -> Pose3d: ...
@overload
def get_alliance(x: Rotation2d) -> Rotation2d: ...
@overload
def get_alliance(x: Translation2d) -> Translation2d: ...
@overload
def get_alliance(x: Translation3d) -> Translation3d: ...

def get_alliance(x):
    if isinstance(x, Pose2d):
        return Pose2d(get_alliance(x.translation()), get_alliance(x.rotation()))

    if isinstance(x, Pose3d):
        return Pose3d(get_alliance(x.translation()), get_alliance(x.rotation()))

    if isinstance(x, Rotation2d):
        return x.rotateBy(_ROTATE_180) if should_flip() else x

    if isinstance(x, Translation2d):
        return Translation2d(get_x(x.X()), get_y(x.Y()))

    if isinstance(x, Translation3d):
        return Translation3d(get_x(x.X()), get_y(x.Y()), x.Z())

    raise TypeError(f"Unsupported type: {type(x)}")