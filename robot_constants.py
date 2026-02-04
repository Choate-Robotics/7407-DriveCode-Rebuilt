from phoenix6 import units
from wpimath.controller import PIDController
from wpimath.geometry import Translation2d

#heading pid tolerance ALL PLACEHOLDERS
aiming_pid_p = 10
aiming_pid_i = 0
aiming_pid_d = 0

drive_at_angle_tolerance: units.radian = 0.05 #placeholder value, ~2.86 degrees

# passing setpoints
pass_target_1: Translation2d = Translation2d(2.0, 1.0) #placeholder values
pass_target_2: Translation2d = Translation2d(5.0, 2.0) #placeholder values

pass_offset = 0.5 #placeholder values