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
    Translation3d(inchesToMeters(-12.418), inchesToMeters(8.921), inchesToMeters(10.525)),
    Rotation3d(0, math.radians(-30), math.radians(-175))
)
front_cam_transform = Transform3d(
    Translation3d(inchesToMeters(-4.231), inchesToMeters(-10.909), inchesToMeters(20.185)),
    Rotation3d(0, math.radians(-31.884), math.radians(10))
)

#odometry
odometry_tag_distance = 3

tower_drivetrain_angle = Rotation2d(0)
tower_flywheel_velocity = 43
tower_hood_angle = 28/360

hub_drivetrain_angle = Rotation2d(0)
hub_flywheel_velocity = 38
hub_hood_angle = 15/360

leftpass_drivetrain_angle = Rotation2d(-160) # PLACEHOLDER
leftpass_flywheel_velocity = 0 # PLACEHOLDER
leftpass_hood_angle = 0 # PLACEHOLDER

rightpass_drivetrain_angle = Rotation2d(160) # PLACEHOLDER
rightpass_flywheel_velocity = 0 # PLACEHOLDER
rightpass_hood_angle = 0 # PLACEHOLDER
