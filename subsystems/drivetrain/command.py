import commands2
from .constants import *
import math

from .command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from utils import alliance_flip_util, field_constants, math_utils, shooter_utils

class AimDrivetrain(commands2.Command):
    def __init__(self, subsystem: CommandSwerveDrivetrain, controller: commands2.button.CommandXboxController):
        super().__init__()

        self.drivetrain = subsystem
        self.controller = controller
        self._brake = swerve.requests.SwerveDriveBrake()
        self._aim_at = swerve.requests.FieldCentricFacingAngle().with_heading_pid(
            aiming_pid_p,
            aiming_pid_i,
            aiming_pid_d
        )

        self.addRequirements(self.drivetrain)

    def initialize(self):
        pass

    def execute(self):
        """
        1. Calculate target angle based on ROBOT POSE and position on field -> determine passing or shooting
        2. Calculate v_x and v_y based on controller inputs
        3. If speed is too high, drive at angle
        4. Check if drivetrain is facing angle within tolerance
        5. If facing angle and speed is 0, apply brake
        6. Else: drive at angle
        """
        if alliance_flip_util.get_x(self.drivetrain.get_pose().X()) < field_constants.LinesVertical.ALLIANCE_ZONE:
            self.target_angle = alliance_flip_util.get_alliance(shooter_utils.angle_aim_to_target(
                self.drivetrain.get_pose(),
                alliance_flip_util.get_alliance(field_constants.Hub.INNER_CENTER_POINT),
            ))

        else:
            self.target_angle = alliance_flip_util.get_alliance(shooter_utils.angle_aim_to_target(
                self.drivetrain.get_pose(),
                shooter_utils.get_pass_setpoint(self.drivetrain.get_pose())
            ))

        self.v_x = math_utils.curve(-self.controller.getLeftY(), deadband) * max_speed
        self.v_y = math_utils.curve(-self.controller.getLeftX(), deadband, curve) * max_speed

        self.cmd_speed = math.hypot(self.v_x, self.v_y)

        if self.drivetrain.is_facing_angle(self.target_angle.radians()) and self.cmd_speed == 0:
            self.drivetrain.set_control(self._brake)

        elif self.cmd_speed > 0 or not self.drivetrain.is_facing_angle(self.target_angle.radians()):
            self.drivetrain.set_control(
                self._aim_at.with_target_direction(self.target_angle)
                .with_velocity_x(self.v_x)
                .with_velocity_y(self.v_y)
            )
        
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        pass


class SnakeMode(commands2.Command):
    def __init__(self, subsystem: CommandSwerveDrivetrain, controller: commands2.button.CommandXboxController):
        super().__init__()

        self.drivetrain = subsystem
        self.controller = controller
        self._brake = swerve.requests.SwerveDriveBrake()
        self.addRequirements(self.drivetrain)

    def initialize(self):
        pass

    def execute(self):
        direction = self.controller.getLeftX()
        speed = self.controller.getLeftY()
        magnitude = math.hypot(direction, speed)
        angle = math.atan2(speed, direction )
    """
    driving logic: I think I'd need the CAN ID's of the back to swerve pods to apply a constant direction. 
    The turning is going to be tricky as I think we want it to mirror how a car drives, but to do that we need something called Ackerman Steering
    The outside wheel turns less than the inside wheel. Idk how to find the ratio between the angle of the two tires       
    """
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        pass