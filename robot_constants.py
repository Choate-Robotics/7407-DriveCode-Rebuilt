from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Rotation2d
from wpimath.units import inchesToMeters
import math

LOGGING = True


# photonvision 
left_cam_name = "left_cam"
right_cam_name = "right_cam"
back_cam_name = "back_cam"
front_cam_name = "front_cam"

left_cam_transform = Transform3d(
    Translation3d(inchesToMeters(-7.671), inchesToMeters(13.967), inchesToMeters(13.984)),
    Rotation3d(0, math.radians(-20), math.radians(75))
)
right_cam_transform = Transform3d(
    Translation3d(inchesToMeters(-7.671), inchesToMeters(-13.967), inchesToMeters(13.984)),
    Rotation3d(0, math.radians(-20), math.radians(-75))
)
back_cam_transform = Transform3d(
    Translation3d(0, 0, 0),
    Rotation3d(0, 0, 0)
)
front_cam_transform = Transform3d(
    Translation3d(inchesToMeters(-4.231), inchesToMeters(10.909), inchesToMeters(20.185)),
    Rotation3d(0, math.radians(-31.537), math.radians(-10))
)

#odometry
odometry_tag_distance = 3

tower_drivetrain_angle = Rotation2d(0) # PLACEHOLDER
tower_flywheel_velocity = 0 # PLACEHOLDER
tower_hood_angle = 0 # PLACEHOLDER

hub_drivetrain_angle = Rotation2d(0) # PLACEHOLDER
hub_flywheel_velocity = 0 # PLACEHOLDER
hub_hood_angle = 0 # PLACEHOLDER

leftpass_drivetrain_angle = Rotation2d(-160) # PLACEHOLDER
leftpass_flywheel_velocity = 0 # PLACEHOLDER
leftpass_hood_angle = 0 # PLACEHOLDER

rightpass_drivetrain_angle = Rotation2d(160) # PLACEHOLDER
rightpass_flywheel_velocity = 0 # PLACEHOLDER
rightpass_hood_angle = 0 # PLACEHOLDER
