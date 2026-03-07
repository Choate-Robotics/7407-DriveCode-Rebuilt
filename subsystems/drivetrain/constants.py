from phoenix6 import units
from wpimath.geometry import Translation2d
from generated.tuner_constants import TunerConstants
from wpimath.units import rotationsToRadians

max_speed = (
    1.0 * TunerConstants.speed_at_12_volts
)  # speed_at_12_volts desired top speed
max_angular_rate = rotationsToRadians(
    1.5
)  # 3/4 of a rotation per second max angular velocity

deadband = 0.05  # deadband for controller inputs
curve = 2  # curve exponent for controller inputs

#heading pid tolerance ALL PLACEHOLDERS
aiming_kP = 7
aiming_kI = 0
aiming_kD = 0.25

snake_mode_kP = 5
snake_mode_kI = 0
snake_mode_kD = 0

drive_at_angle_tolerance: units.radian = 0.035
drivetrain_shooting_velocity_tolerance: units.meters_per_second = 0.5

autoalign_pid_p = 5
autoalign_pid_i = 0
autoalign_pid_d = 0
