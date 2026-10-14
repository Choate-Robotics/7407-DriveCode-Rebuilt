from phoenix6 import swerve, units
from wpimath.geometry import Translation2d
from generated.tuner_constants import TunerConstants
from wpimath.units import rotationsToRadians, degreesToRadians

max_speed = (
    1.0 * TunerConstants.speed_at_12_volts
)  # speed_at_12_volts desired top speed
max_angular_rate = rotationsToRadians(
    1.5
)  # 3/4 of a rotation per second max angular velocity

deadband = 0.05  # deadband for controller inputs
curve = 1  # curve exponent for controller inputs - linear, matches MoSim's rotation feel

translation_deadband_min = 0.125  # radial deadzone, matches MoSim/Unity's default StickDeadzone
translation_deadband_max = 0.925  # stick magnitude that already counts as full speed

#heading pid tolerance ALL PLACEHOLDERS
# Was kP 7 / kD 0.25: in sim (with wheel friction) that stalled ~1.9 deg short of the
# target. kP 14 / kD 0.35 settles within ~1 deg in 0.3-0.4 s with no overshoot. kI made
# it overshoot up to 6 deg. Re-check on the robot; back off kP if heading oscillates.
aiming_kP = 14
aiming_kI = 0
aiming_kD = 0.35

snake_mode_kP = 5
snake_mode_kI = 0
snake_mode_kD = 0

drive_at_angle_tolerance: units.radian = degreesToRadians(3)

# Gyro failure check: the Pigeon fails by freezing (reading the same value, e.g. zeros)
# while staying connected. A working gyro's yaw always changes while the robot turns, so
# it's flagged only when the wheels (encoder kinematics, independent of the gyro) say the
# robot is turning faster than this and the yaw hasn't changed for gyro_frozen_time.
# Standing still can't trigger it.
gyro_check_min_turn_rate: units.radians_per_second = 0.3
gyro_frozen_time: units.second = 0.25

# Hub aiming: the tolerance is a sideways miss limit at the hub instead of a fixed angle,
# so far shots must point more precisely (~2.3 deg at 2.5 m, ~1.4 deg at 4 m, 1 deg from
# ~6 m), capped at drive_at_angle_tolerance. Passes and fixed-angle presets keep 3 deg.
aim_lateral_tolerance: units.meter = 0.10
aim_min_tolerance: units.radian = degreesToRadians(1.0)
# If the heading controller stalls (not rotating) inside drive_at_angle_tolerance for
# this long, count the aim as settled anyway so a far shot can't wait forever.
aim_stall_time: units.second = 0.5
aim_stall_rate: units.radians_per_second = 0.05
# Only count as aimed once rotation has nearly stopped, so the X-lock never engages
# while the robot is still swinging through the target.
aim_settled_rate: units.radians_per_second = 0.5
drivetrain_shooting_velocity_tolerance: units.meters_per_second = 0.3

# How the drive motors follow auto paths. VELOCITY: each motor controller holds the wheel
# speed itself (1 kHz, using the drive gains in tuner_constants.py) instead of only
# getting a voltage for that speed, so the robot doesn't run past the path under hard
# braking. Set to OPEN_LOOP_VOLTAGE to go back to the old behavior.
auto_drive_request_type = swerve.SwerveModule.DriveRequestType.VELOCITY

autoalign_pid_p = 5
autoalign_pid_i = 0
autoalign_pid_d = 0
