import commands2
import math
from .constants import *
from .command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from utils import alliance_flip_util, field_constants, math_utils, shooter_utils
from wpimath.geometry import Pose2d, Rotation2d
from utils.field_constants import Hub, LinesHorizontal, LinesVertical
import ntcore


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
        self.aim_at = swerve.requests.FieldCentricFacingAngle().with_heading_pid(
            snake_mode_pid_p,
            snake_mode_pid_i,
            snake_mode_pid_d
        )
        self.drive = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self.addRequirements(self.drivetrain)

    def initialize(self):
        pass

    def execute(self):
        joystick_y = math_utils.curve(-self.controller.getLeftY(), deadband)
        joystick_x = math_utils.curve(-self.controller.getLeftX(), deadband)
        aiming_joystick_x = math_utils.curve(-self.controller.getRightX(), deadband, curve)
        target_angle = math.degrees(math.atan2(joystick_x,joystick_y))
        
        if aiming_joystick_x != 0:
            self.drivetrain.set_control(
                self.drive.with_velocity_x(
                    joystick_y * max_speed
                )
                .with_velocity_y(
                    joystick_x * max_speed
                ).with_rotational_rate(aiming_joystick_x * max_angular_rate)
            )

        elif aiming_joystick_x == 0 and joystick_y == 0 and joystick_x == 0:
            self.drivetrain.set_control(swerve.requests.Idle())

        else: 
            self.drivetrain.set_control(
            self.aim_at
                .with_target_direction(Rotation2d.fromDegrees(target_angle))
                .with_velocity_x(joystick_y * max_speed)
                .with_velocity_y(joystick_x * max_speed)  
        )            

    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        pass

class AutoAlign(commands2.Command):
    def __init__(self, subsystem: CommandSwerveDrivetrain, controller: commands2.button.CommandXboxController):
        super().__init__()
        self.drivetrain = subsystem
        self.controller = controller
        self.aim_at = swerve.requests.FieldCentricFacingAngle().with_heading_pid(
            autoalign_pid_p,
            autoalign_pid_i,
            autoalign_pid_d
        )
        self.joystick_y = math_utils.curve(-self.controller.getLeftY(), deadband)
        self.joystick_x = math_utils.curve(-self.controller.getLeftX(), deadband)
        #prob not gonna work cause i still dont understand how the field constants are defined 
        left_trench_center = Translation2d(
            LinesVertical.HUB_CENTER,
            (LinesHorizontal.LEFT_TRENCH_OPEN_END + LinesHorizontal.LEFT_TRENCH_OPEN_START) / 2
        )

        right_trench_center = Translation2d(
            LinesVertical.HUB_CENTER,
            (LinesHorizontal.RIGHT_TRENCH_OPEN_START + LinesHorizontal.RIGHT_TRENCH_OPEN_END) / 2
        )

        left_bump_center = Translation2d(
            (Hub.FAR_LEFT_CORNER.X() + Hub.NEAR_LEFT_CORNER.X()) / 2,
            (LinesHorizontal.LEFT_BUMP_START + LinesHorizontal.LEFT_BUMP_END) / 2
        )

        right_bump_center = Translation2d(
            (Hub.FAR_RIGHT_CORNER.X() + Hub.NEAR_RIGHT_CORNER.X()) / 2,
            (LinesHorizontal.RIGHT_BUMP_START + LinesHorizontal.RIGHT_BUMP_END) / 2
        )
        robot_pose = self.drivetrain.get_pose()
        robot_translation = robot_pose.translation()
        self.drive = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self.left_candidates = {
            "left_trench": robot_translation.distance(left_trench_center),
            "left_bump": robot_translation.distance(left_bump_center),
        }
        self.right_candidates = {
            "right_bump": robot_translation.distance(right_bump_center),
            "right_trench": robot_translation.distance(right_trench_center),
        }

        self.addRequirements(self.drivetrain)
        
    def initialize(self):
        pass

    def execute(self):
        closest_left = min(self.left_candidates["left_trench"], self.left_candidates["left_bump"])
        closest_right = min(self.right_candidates["right_trench"], self.right_candidates["right_bump"])
        if closest_left < closest_right:
            if closest_left == "left_trench":
                self.drivetrain.set_control(
                    self.aim_at
                        .with_target_direction(Rotation2d.fromDegrees(0))
                        .with_velocity_x(self.joystick_y * max_speed)
                        .with_velocity_y(self.joystick_x * max_speed)  
                )
            else:
                self.drivetrain.set_control(
                    self.aim_at
                        .with_target_direction(Rotation2d.fromDegrees(-45))
                        .with_velocity_x(0)
                        .with_velocity_y(0)
                )

        elif closest_right < closest_left:
            if closest_left == "right_trench":
                self.drivetrain.set_control(
                    self.aim_at
                        .with_target_direction(Rotation2d.fromDegrees(0))
                        .with_velocity_x(self.joystick_y * max_speed)
                        .with_velocity_y(self.joystick_x * max_speed)  
                )
            else:
                self.drivetrain.set_control(
                    self.aim_at
                        .with_target_direction(Rotation2d.fromDegrees(45))
                        .with_velocity_x(self.joystick_y * max_speed)
                        .with_velocity_y(self.joystick_x * max_speed)  
                )
        else:
            pass
            
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        pass