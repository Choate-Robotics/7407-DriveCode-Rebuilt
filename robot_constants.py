from phoenix6 import units
from wpimath.controller import PIDController

#heading pid tolerance ALL PLACEHOLDERS
aiming_pid_p = 0
aiming_pid_i = 0
aiming_pid_d = 0


drive_at_angle_tolerance: units.radian = 0.05 #placeholder value, ~2.86 degrees