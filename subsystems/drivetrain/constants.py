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

deadband = 0.1  # deadband for controller inputs
curve = 2  # curve exponent for controller inputs

#heading pid tolerance ALL PLACEHOLDERS
aiming_pid_p = 10
aiming_pid_i = 0
aiming_pid_d = 0

snake_mode_pid_p = 10
snake_mode_pid_i = 0
snake_mode_pid_d = 0


drive_at_angle_tolerance: units.radian = 0.05 #placeholder value, ~2.86 degrees