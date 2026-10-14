import commands2
import math
from .constants import *
from .command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from utils import alliance_flip_util, field_constants, math_utils, shooter_utils
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
from utils.field_constants import Hub, LinesHorizontal, LinesVertical
from wpilib import Timer
import ntcore


def hub_aim_tolerance(distance: float) -> float:
    """Allowed heading error (rad) at this distance from the hub: a fixed sideways miss
    limit, clamped between aim_min_tolerance and drive_at_angle_tolerance."""
    return min(drive_at_angle_tolerance,
               max(aim_min_tolerance, math.atan2(aim_lateral_tolerance, max(distance, 1e-3))))


class AimSettle:
    """True once aimed at the hub: inside the distance-based tolerance, or stalled (not
    rotating for aim_stall_time) inside drive_at_angle_tolerance."""

    def __init__(self):
        self._stalled_since = None

    def reset(self):
        self._stalled_since = None

    def update(self, drivetrain: CommandSwerveDrivetrain, target_angle: float, distance: float) -> bool:
        omega = abs(drivetrain.get_speeds().omega)
        if drivetrain.is_facing_angle(target_angle, hub_aim_tolerance(distance)) and omega < aim_settled_rate:
            self._stalled_since = None
            return True
        not_turning = omega < aim_stall_rate
        if drivetrain.is_facing_angle(target_angle) and not_turning:
            if self._stalled_since is None:
                self._stalled_since = Timer.getFPGATimestamp()
            return Timer.getFPGATimestamp() - self._stalled_since >= aim_stall_time
        self._stalled_since = None
        return False


class AimDrivetrain(commands2.Command):
    def __init__(self, subsystem: CommandSwerveDrivetrain, controller: commands2.button.CommandXboxController):
        super().__init__()

        self.drivetrain = subsystem
        self.controller = controller
        self._brake = swerve.requests.SwerveDriveBrake()
        self._aim_at = swerve.requests.FieldCentricFacingAngle().with_heading_pid(
            aiming_kP,
            aiming_kI,
            aiming_kD
        )

        self.addRequirements(self.drivetrain)

        self.table = ntcore.NetworkTableInstance.getDefault().getTable("Shot Tuner")
        self.pose_pub = self.table.getStructTopic("Target pose", Pose2d).publish()
        self.angle_pub = self.table.getDoubleTopic("Angle").publish()
        self.distance_pub = self.table.getDoubleTopic("distance to target").publish()

        self.ready_pub = self.table.getBooleanTopic("Ready to shoot").publish()
        self.aim_error_pub = self.table.getDoubleTopic("Aim error deg").publish()
        self.aim_tolerance_pub = self.table.getDoubleTopic("Aim tolerance deg").publish()

        self.target = None
        self.settle = AimSettle()

    def initialize(self):
        self.settle.reset()

    def execute(self):
        self.pose = self.drivetrain.get_pose()
        self.passing = alliance_flip_util.get_x(self.pose.X()) >= field_constants.LinesVertical.NEUTRAL_ZONE_NEAR

        if not self.passing:
            self.target = alliance_flip_util.get_alliance(
                field_constants.Hub.INNER_CENTER_POINT
            ).toTranslation2d()
        else:
            self.target = shooter_utils.get_pass_setpoint(self.pose)

        raw_target_angle = shooter_utils.angle_aim_to_target(
            self.pose,
            self.target,
        )
        self.target_angle = alliance_flip_util.get_alliance(raw_target_angle)

        self.distance = self.pose.translation().distance(self.target)
        self.distance_pub.set(self.distance)
        # self.pose_pub.set(Pose2d(self.target, Rotation2d()))
        # self.angle_pub.set(self.target_angle.degrees())

        self.v_x, self.v_y = math_utils.radial_curve(
            -self.controller.getLeftY(),
            -self.controller.getLeftX(),
            translation_deadband_min,
            translation_deadband_max,
        )
        self.v_x *= max_speed
        self.v_y *= max_speed

        self.cmd_speed = math.hypot(self.v_x, self.v_y)
        field_angle = alliance_flip_util.get_alliance(self.target_angle).radians()
        if self.passing:  # passes target a big zone: plain 3 deg tolerance
            self.settle.reset()
            self.is_facing_angle = self.drivetrain.is_facing_angle(field_angle)
        else:
            self.is_facing_angle = self.settle.update(self.drivetrain, field_angle, self.distance)
            self.aim_error_pub.set(math.degrees(math_utils.bounded_angle_diff(self.pose.rotation().radians(), field_angle)))
            self.aim_tolerance_pub.set(math.degrees(hub_aim_tolerance(self.distance)))

        # Freeze only for scoring, never for passing
        if (not self.passing) and self.is_facing_angle and self.cmd_speed == 0:
            self.drivetrain.set_control(self._brake)
        else:
            self.drivetrain.set_control(
                self._aim_at.with_target_direction(self.target_angle)
                .with_velocity_x(self.v_x)
                .with_velocity_y(self.v_y)
            )

        self.drivetrain.ready_to_shoot = (
            self.cmd_speed < drivetrain_shooting_velocity_tolerance
            and self.is_facing_angle
            and not self.passing
        )

    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        self.drivetrain.ready_to_shoot = False

class AimDrivetrainAuto(commands2.Command):
    def __init__(self, subsystem: CommandSwerveDrivetrain):
        super().__init__()

        self.drivetrain = subsystem
        self._brake = swerve.requests.SwerveDriveBrake()
        self._aim_at = swerve.requests.FieldCentricFacingAngle().with_heading_pid(
            aiming_kP,
            aiming_kI,
            aiming_kD
        )

        self.addRequirements(self.drivetrain)

        # self.table = ntcore.NetworkTableInstance.getDefault().getTable("Shot Tuner")
        # self.pose_pub = self.table.getStructTopic("Target pose", Pose2d).publish()
        # self.angle_pub = self.table.getDoubleTopic("Angle").publish()
        # self.distance_pub = self.table.getDoubleTopic("distance to target").publish()

        # self.ready_pub = self.table.getBooleanTopic("Ready to shoot").publish()

        self.target = None
        self.settle = AimSettle()

    def initialize(self):
        self.settle.reset()

    def execute(self):
        """
        1. Calculate target angle based on ROBOT POSE and position on field -> determine passing or shooting
        2. Calculate v_x and v_y based on controller inputs
        3. If speed is too high, drive at angle
        4. Check if drivetrain is facing angle within tolerance
        5. If facing angle and speed is 0, apply brake
        6. Else: drive at angle
        """
        self.pose = self.drivetrain.get_pose()
        if alliance_flip_util.get_x(self.pose.X()) < field_constants.LinesVertical.ALLIANCE_ZONE:
            self.target = alliance_flip_util.get_alliance(field_constants.Hub.INNER_CENTER_POINT).toTranslation2d()
            self.target_angle = alliance_flip_util.get_alliance(shooter_utils.angle_aim_to_target(
                self.pose,
                self.target,
            ))
            # self.pose_pub.set(Pose2d(self.target, Rotation2d()))

        else:
            self.target = shooter_utils.get_pass_setpoint(self.pose)
            self.target_angle = alliance_flip_util.get_alliance(shooter_utils.angle_aim_to_target(
                self.pose,
                self.target
            ))
            # self.pose_pub.set(Pose2d(self.target, Rotation2d()))

        field_angle = alliance_flip_util.get_alliance(self.target_angle).radians()
        if alliance_flip_util.get_x(self.pose.X()) < field_constants.LinesVertical.ALLIANCE_ZONE:
            distance = self.pose.translation().distance(self.target)
            self.is_facing_angle = self.settle.update(self.drivetrain, field_angle, distance)
        else:  # passing: plain 3 deg tolerance
            self.settle.reset()
            self.is_facing_angle = self.drivetrain.is_facing_angle(field_angle)

        self.drivetrain.set_control(
            self._aim_at.with_target_direction(self.target_angle)
            .with_velocity_x(0)
            .with_velocity_y(0)
        )

        self.drivetrain.ready_to_shoot = self.is_facing_angle

    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        self.drivetrain.ready_to_shoot = False

class DriveAtAngle(commands2.Command):
    def __init__(self, subsystem: CommandSwerveDrivetrain, controller: commands2.button.CommandXboxController, target_angle: Rotation2d):
        super().__init__()

        self.drivetrain = subsystem
        self.controller = controller
        self._brake = swerve.requests.SwerveDriveBrake()
        self._aim_at = swerve.requests.FieldCentricFacingAngle().with_heading_pid(
            aiming_kP,
            aiming_kI,
            aiming_kD
        )

        self.addRequirements(self.drivetrain)

        self.base_target_angle = target_angle
        self.target_angle = target_angle

    def initialize(self):
        self.target_angle = alliance_flip_util.get_alliance(self.base_target_angle)

    def execute(self):
        """
        1. Calculate v_x and v_y based on controller inputs
        2. If speed is too high, drive at angle
        3. Check if drivetrain is facing angle within tolerance
        4. If facing angle and speed is 0, apply brake
        5. Else: drive at angle
        """

        self.v_x, self.v_y = math_utils.radial_curve(
            -self.controller.getLeftY(),
            -self.controller.getLeftX(),
            translation_deadband_min,
            translation_deadband_max,
        )
        self.v_x *= max_speed
        self.v_y *= max_speed

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


class SnakeMode(commands2.Command):
    def __init__(self, subsystem: CommandSwerveDrivetrain, controller: commands2.button.CommandXboxController):
        super().__init__()
        self.drivetrain = subsystem
        self.controller = controller
        self.aim_at = swerve.requests.FieldCentricFacingAngle().with_heading_pid(
            snake_mode_kP,
            snake_mode_kI,
            snake_mode_kD
        )
        self.drive = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self.addRequirements(self.drivetrain)

    def initialize(self):
        pass

    def execute(self):
        """"
        Gets the input from the left controller to get x and y velocities for the robot
        Then, gets the aiming direction from the right controller
        Uses aformentioned inputs to drive the robot like a car
        e.g. drives in the direction the robot is facing
        """
        joystick_y, joystick_x = math_utils.radial_curve(
            -self.controller.getLeftY(),
            -self.controller.getLeftX(),
            translation_deadband_min,
            translation_deadband_max,
        )
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
        self.left_centers = {
            "left_trench": left_trench_center,
            "left_bump": left_bump_center,
        }
        self.right_centers = {
            "right_trench": right_trench_center,
            "right_bump": right_bump_center,
        }
        self.target_angle = Rotation2d.fromDegrees(0)
        self.addRequirements(self.drivetrain)
        

    def initialize(self):
        """
        Calculates the closest field element based on current robot pose and field util
        If closest element is a trench, the robot will align to be perpendicular to the bottom edge of the field
        Otherwise, if the closest element is a bump, the robot aligns to be at a 45 degree angle
        """
        robot_translation = self.drivetrain.get_pose().translation()
        self.closest_left = min(self.left_centers, key=lambda k: robot_translation.distance(self.left_centers[k]))
        self.closest_right = min(self.right_centers, key=lambda k: robot_translation.distance(self.right_centers[k]))
        self.left_dist = robot_translation.distance(self.left_centers[self.closest_left])
        self.right_dist = robot_translation.distance(self.right_centers[self.closest_right])
        
        

    def execute(self):
        self.robot_rotation = self.drivetrain.get_pose().rotation().degrees()
        joystick_y, joystick_x = math_utils.radial_curve(
            -self.controller.getLeftY(),
            -self.controller.getLeftX(),
            translation_deadband_min,
            translation_deadband_max,
        )

        if self.left_dist <= self.right_dist:
            if self.closest_left == "left_trench":
                self.target_angle = Rotation2d.fromDegrees(round(self.robot_rotation / 90) * 90)
            else:
                self.target_angle = self.drivetrain.snap_bump_angle(self.robot_rotation)
        else:
            if self.closest_right == "right_trench":
                self.target_angle = Rotation2d.fromDegrees(round(self.robot_rotation / 90) * 90)
            else:
                self.target_angle = self.drivetrain.snap_bump_angle(self.robot_rotation)

        self.drivetrain.set_control(
                self.aim_at
                    .with_target_direction(self.target_angle)
                    .with_velocity_x(joystick_y * max_speed)
                    .with_velocity_y(joystick_x * max_speed)
            )

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass