"""
These classes are inspireed by the FieldConstants class in
    6328's 2026 code.
    https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/FieldConstants.java
    // Copyright (c) 2025-2026 FRC 6328
    // http://github.com/Mechanical-Advantage
    //
    // Use of this source code is governed by an MIT-style
    // license that can be found in the LICENSE file at
    // the root directory of this project.

Contains information for location of field element and other useful reference points.

NOTE: All constants are defined relative to the field coordinate system, and from the
perspective of the blue alliance station
"""

from wpimath.geometry import Translation2d, Translation3d, Pose2d, Rotation2d
from wpimath.units import inchesToMeters, meters
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
import math
from wpilib import DriverStation


# Load the AprilTag layout (equivalent to AprilTagLayoutType.OFFICIAL.getLayout())
_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)

# AprilTag related constants
APRILTAG_COUNT = len(_layout.getTags())
APRILTAG_WIDTH = inchesToMeters(6.5)

# Field dimensions
FIELD_LENGTH = _layout.getFieldLength()
FIELD_WIDTH = _layout.getFieldWidth()

# passing setpoints
pass_target_1: Translation2d = Translation2d(3, 2)
pass_target_2: Translation2d = Translation2d(1, 1)

pass_offset: meters = 0.75

class Hub:
    """Hub related constants"""

    # Dimensions
    WIDTH = inchesToMeters(47.0)
    HEIGHT = inchesToMeters(72.0)  # includes the catcher at the top
    INNER_WIDTH = inchesToMeters(41.7)
    INNER_HEIGHT = inchesToMeters(56.5)

    # Relevant reference points on alliance side
    TOP_CENTER_POINT = Translation3d(
        _layout.getTagPose(26).X() + WIDTH / 2.0,
        FIELD_WIDTH / 2.0,
        HEIGHT
    )
    INNER_CENTER_POINT = Translation3d(
        _layout.getTagPose(26).X() + WIDTH / 2.0,
        FIELD_WIDTH / 2.0,
        INNER_HEIGHT
    )

    NEAR_LEFT_CORNER = Translation2d(
        TOP_CENTER_POINT.X() - WIDTH / 2.0,
        FIELD_WIDTH / 2.0 + WIDTH / 2.0
    )
    NEAR_RIGHT_CORNER = Translation2d(
        TOP_CENTER_POINT.X() - WIDTH / 2.0,
        FIELD_WIDTH / 2.0 - WIDTH / 2.0
    )
    FAR_LEFT_CORNER = Translation2d(
        TOP_CENTER_POINT.X() + WIDTH / 2.0,
        FIELD_WIDTH / 2.0 + WIDTH / 2.0
    )
    FAR_RIGHT_CORNER = Translation2d(
        TOP_CENTER_POINT.X() + WIDTH / 2.0,
        FIELD_WIDTH / 2.0 - WIDTH / 2.0
    )

    # Relevant reference points on the opposite side
    OPP_TOP_CENTER_POINT = Translation3d(
        _layout.getTagPose(4).X() + WIDTH / 2.0,
        FIELD_WIDTH / 2.0,
        HEIGHT
    )
    OPP_NEAR_LEFT_CORNER = Translation2d(
        OPP_TOP_CENTER_POINT.X() - WIDTH / 2.0,
        FIELD_WIDTH / 2.0 + WIDTH / 2.0
    )
    OPP_NEAR_RIGHT_CORNER = Translation2d(
        OPP_TOP_CENTER_POINT.X() - WIDTH / 2.0,
        FIELD_WIDTH / 2.0 - WIDTH / 2.0
    )
    OPP_FAR_LEFT_CORNER = Translation2d(
        OPP_TOP_CENTER_POINT.X() + WIDTH / 2.0,
        FIELD_WIDTH / 2.0 + WIDTH / 2.0
    )
    OPP_FAR_RIGHT_CORNER = Translation2d(
        OPP_TOP_CENTER_POINT.X() + WIDTH / 2.0,
        FIELD_WIDTH / 2.0 - WIDTH / 2.0
    )

    # Hub faces
    NEAR_FACE = _layout.getTagPose(26).toPose2d()
    FAR_FACE = _layout.getTagPose(20).toPose2d()
    RIGHT_FACE = _layout.getTagPose(18).toPose2d()
    LEFT_FACE = _layout.getTagPose(21).toPose2d()


class LinesVertical:
    """
    Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
    """

    CENTER = FIELD_LENGTH / 2.0

    STARTING = _layout.getTagPose(26).X()
    ALLIANCE_ZONE = STARTING

    HUB_CENTER = _layout.getTagPose(26).X() + Hub.WIDTH / 2.0
    OPP_HUB_CENTER = _layout.getTagPose(4).X() + Hub.WIDTH / 2.0

    NEUTRAL_ZONE_NEAR = CENTER - inchesToMeters(120)
    NEUTRAL_ZONE_FAR = CENTER + inchesToMeters(120)

    OPP_ALLIANCE_ZONE = _layout.getTagPose(10).X()


class LeftBump:
    """Left Bump related constants"""

    # Dimensions
    WIDTH = inchesToMeters(73.0)
    HEIGHT = inchesToMeters(6.513)
    DEPTH = inchesToMeters(44.4)

    # Relevant reference points on alliance side
    NEAR_LEFT_CORNER = Translation2d(
        LinesVertical.HUB_CENTER - WIDTH / 2,
        inchesToMeters(255)
    )
    NEAR_RIGHT_CORNER = Hub.NEAR_LEFT_CORNER
    FAR_LEFT_CORNER = Translation2d(
        LinesVertical.HUB_CENTER + WIDTH / 2,
        inchesToMeters(255)
    )
    FAR_RIGHT_CORNER = Hub.FAR_LEFT_CORNER

    # Relevant reference points on opposing side
    OPP_NEAR_LEFT_CORNER = Translation2d(
        LinesVertical.HUB_CENTER - WIDTH / 2,
        inchesToMeters(255)
    )
    OPP_NEAR_RIGHT_CORNER = Hub.OPP_NEAR_LEFT_CORNER
    OPP_FAR_LEFT_CORNER = Translation2d(
        LinesVertical.HUB_CENTER + WIDTH / 2,
        inchesToMeters(255)
    )
    OPP_FAR_RIGHT_CORNER = Hub.OPP_FAR_LEFT_CORNER


class RightBump:
    """Right Bump related constants"""

    # Dimensions
    WIDTH = inchesToMeters(73.0)
    HEIGHT = inchesToMeters(6.513)
    DEPTH = inchesToMeters(44.4)

    # Relevant reference points on alliance side
    NEAR_LEFT_CORNER = Translation2d(
        LinesVertical.HUB_CENTER + WIDTH / 2,
        inchesToMeters(255)
    )
    NEAR_RIGHT_CORNER = Hub.NEAR_LEFT_CORNER
    FAR_LEFT_CORNER = Translation2d(
        LinesVertical.HUB_CENTER - WIDTH / 2,
        inchesToMeters(255)
    )
    FAR_RIGHT_CORNER = Hub.FAR_LEFT_CORNER

    # Relevant reference points on opposing side
    OPP_NEAR_LEFT_CORNER = Translation2d(
        LinesVertical.HUB_CENTER + WIDTH / 2,
        inchesToMeters(255)
    )
    OPP_NEAR_RIGHT_CORNER = Hub.OPP_NEAR_LEFT_CORNER
    OPP_FAR_LEFT_CORNER = Translation2d(
        LinesVertical.HUB_CENTER - WIDTH / 2,
        inchesToMeters(255)
    )
    OPP_FAR_RIGHT_CORNER = Hub.OPP_FAR_LEFT_CORNER


class LinesHorizontal:
    """
    Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)

    NOTE: The field element start and end are always left to right from the perspective of the
    alliance station
    """

    CENTER = FIELD_WIDTH / 2.0

    # Right of hub
    RIGHT_BUMP_START = Hub.NEAR_RIGHT_CORNER.Y()
    RIGHT_BUMP_END = RIGHT_BUMP_START - RightBump.WIDTH
    RIGHT_TRENCH_OPEN_START = RIGHT_BUMP_END - inchesToMeters(12.0)
    RIGHT_TRENCH_OPEN_END = 0.0

    # Left of hub
    LEFT_BUMP_END = Hub.NEAR_LEFT_CORNER.Y()
    LEFT_BUMP_START = LEFT_BUMP_END + LeftBump.WIDTH
    LEFT_TRENCH_OPEN_END = LEFT_BUMP_START + inchesToMeters(12.0)
    LEFT_TRENCH_OPEN_START = FIELD_WIDTH


class LeftTrench:
    """Left Trench related constants"""

    # Dimensions
    WIDTH = inchesToMeters(65.65)
    DEPTH = inchesToMeters(47.0)
    HEIGHT = inchesToMeters(40.25)
    OPENING_WIDTH = inchesToMeters(50.34)
    OPENING_HEIGHT = inchesToMeters(22.25)

    # Relevant reference points on alliance side
    OPENING_TOP_LEFT = Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT)
    OPENING_TOP_RIGHT = Translation3d(
        LinesVertical.HUB_CENTER, FIELD_WIDTH - OPENING_WIDTH, OPENING_HEIGHT
    )

    # Relevant reference points on opposing side
    OPP_OPENING_TOP_LEFT = Translation3d(
        LinesVertical.OPP_HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT
    )
    OPP_OPENING_TOP_RIGHT = Translation3d(
        LinesVertical.OPP_HUB_CENTER, FIELD_WIDTH - OPENING_WIDTH, OPENING_HEIGHT
    )


class RightTrench:
    """Right Trench related constants"""

    # Dimensions
    WIDTH = inchesToMeters(65.65)
    DEPTH = inchesToMeters(47.0)
    HEIGHT = inchesToMeters(40.25)
    OPENING_WIDTH = inchesToMeters(50.34)
    OPENING_HEIGHT = inchesToMeters(22.25)

    # Relevant reference points on alliance side
    OPENING_TOP_LEFT = Translation3d(LinesVertical.HUB_CENTER, OPENING_WIDTH, OPENING_HEIGHT)
    OPENING_TOP_RIGHT = Translation3d(LinesVertical.HUB_CENTER, 0, OPENING_HEIGHT)

    # Relevant reference points on opposing side
    OPP_OPENING_TOP_LEFT = Translation3d(
        LinesVertical.OPP_HUB_CENTER, OPENING_WIDTH, OPENING_HEIGHT
    )
    OPP_OPENING_TOP_RIGHT = Translation3d(LinesVertical.OPP_HUB_CENTER, 0, OPENING_HEIGHT)


class Tower:
    """Tower related constants"""

    # Dimensions
    WIDTH = inchesToMeters(49.25)
    DEPTH = inchesToMeters(45.0)
    HEIGHT = inchesToMeters(78.25)
    INNER_OPENING_WIDTH = inchesToMeters(32.250)
    FRONT_FACE_X = inchesToMeters(43.51)

    UPRIGHT_HEIGHT = inchesToMeters(72.1)

    # Rung heights from the floor
    LOW_RUNG_HEIGHT = inchesToMeters(27.0)
    MID_RUNG_HEIGHT = inchesToMeters(45.0)
    HIGH_RUNG_HEIGHT = inchesToMeters(63.0)

    # Relevant reference points on alliance side
    CENTER_POINT = Translation2d(FRONT_FACE_X, _layout.getTagPose(31).Y())
    LEFT_UPRIGHT = Translation2d(
        FRONT_FACE_X,
        _layout.getTagPose(31).Y() + INNER_OPENING_WIDTH / 2 + inchesToMeters(0.75)
    )
    RIGHT_UPRIGHT = Translation2d(
        FRONT_FACE_X,
        _layout.getTagPose(31).Y() - INNER_OPENING_WIDTH / 2 - inchesToMeters(0.75)
    )

    # Relevant reference points on opposing side
    OPP_CENTER_POINT = Translation2d(FIELD_LENGTH - FRONT_FACE_X, _layout.getTagPose(15).Y())
    OPP_LEFT_UPRIGHT = Translation2d(
        FIELD_LENGTH - FRONT_FACE_X,
        _layout.getTagPose(15).Y() + INNER_OPENING_WIDTH / 2 + inchesToMeters(0.75)
    )
    OPP_RIGHT_UPRIGHT = Translation2d(
        FIELD_LENGTH - FRONT_FACE_X,
        _layout.getTagPose(15).Y() - INNER_OPENING_WIDTH / 2 - inchesToMeters(0.75)
    )


class Depot:
    """Depot related constants"""

    # Dimensions
    WIDTH = inchesToMeters(42.0)
    DEPTH = inchesToMeters(27.0)
    HEIGHT = inchesToMeters(1.125)
    DISTANCE_FROM_CENTER_Y = inchesToMeters(75.93)

    # Relevant reference points on alliance side
    DEPOT_CENTER = Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y, HEIGHT)
    LEFT_CORNER = Translation3d(
        DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y + (WIDTH / 2), HEIGHT
    )
    RIGHT_CORNER = Translation3d(
        DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y - (WIDTH / 2), HEIGHT
    )


class Outpost:
    """Outpost related constants"""

    # Dimensions
    WIDTH = inchesToMeters(31.8)
    OPENING_DISTANCE_FROM_FLOOR = inchesToMeters(28.1)
    HEIGHT = inchesToMeters(7.0)

    # Relevant reference points on alliance side
    CENTER_POINT = Translation2d(0, _layout.getTagPose(29).Y())
