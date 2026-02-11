import commands2
from .constants import *
import math

from .command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from utils import alliance_flip_util, field_constants, math_utils, shooter_utils
from wpimath.geometry import Rotation2d

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
        self.is_facing_angle = self.drivetrain.is_facing_angle(self.target_angle.radians())

        if self.is_facing_angle and self.cmd_speed == 0:
            self.drivetrain.set_control(self._brake)

        else:
            self.drivetrain.set_control(
                self._aim_at.with_target_direction(self.target_angle)
                .with_velocity_x(self.v_x)
                .with_velocity_y(self.v_y)
            )

        self.drivetrain.ready_to_shoot = self.cmd_speed < drivetrain_shooting_velocity_tolerance and self.is_facing_angle
        
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        pass

class DriveAtAngle(commands2.Command):
    def __init__(self, subsystem: CommandSwerveDrivetrain, controller: commands2.button.CommandXboxController, target_angle: Rotation2d):
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
        self.target_angle = target_angle

    def initialize(self):
        self.target_angle = alliance_flip_util.get_alliance(self.target_angle)

    def execute(self):
        """
        1. Calculate v_x and v_y based on controller inputs
        2. If speed is too high, drive at angle
        3. Check if drivetrain is facing angle within tolerance
        4. If facing angle and speed is 0, apply brake
        5. Else: drive at angle
        """

        self.v_x = math_utils.curve(-self.controller.getLeftY(), deadband) * max_speed
        self.v_y = math_utils.curve(-self.controller.getLeftX(), deadband, curve) * max_speed

        self.cmd_speed = math.hypot(self.v_x, self.v_y)
        self.is_facing_angle = self.drivetrain.is_facing_angle(self.target_angle.radians())

        if self.is_facing_angle and self.cmd_speed == 0:
            self.drivetrain.set_control(self._brake)

        else:
            self.drivetrain.set_control(
                self._aim_at.with_target_direction(self.target_angle)
                .with_velocity_x(self.v_x)
                .with_velocity_y(self.v_y)
            )

        self.drivetrain.ready_to_shoot = self.cmd_speed < drivetrain_shooting_velocity_tolerance and self.is_facing_angle
        
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        pass