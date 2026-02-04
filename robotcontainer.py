#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import commands2
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpilib import DriverStation, SendableChooser, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

import math
import autos
from utils import shooter_utils, alliance_flip_util, field_constants
import robot_constants

from subsystems import *


def curve(x, d, c=1):
    if abs(x) < d:
        return 0
    elif x < 0:
        return -1 * math.pow((-1 * (x + d) / (1 - d)), c)
    return math.pow(((x - d) / (1 - d)), c)

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            1.5
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._aim_at = swerve.requests.FieldCentricFacingAngle().with_heading_pid(
            robot_constants.aiming_pid_p,
            robot_constants.aiming_pid_i,
            robot_constants.aiming_pid_d
        )

        self._logger = Telemetry(self._max_speed)

        self.driver_controller = CommandXboxController(0)
        self.operator_controller = CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        self.auto_selection = SendableChooser()
        self.auto_selection.setDefaultOption("Drive Forward", autos.leave(self))

        SmartDashboard.putData("Auto", self.auto_selection)

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        curve(-self.driver_controller.getLeftY(), 0.1) * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        curve(-self.driver_controller.getLeftX(), 0.1, 2) * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self.driver_controller.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # shooting masterpiece
        self.driver_controller.rightTrigger().whileTrue(
            commands2.ConditionalCommand(
                # hub shooting
                commands2.ConditionalCommand(
                    self.drivetrain.apply_request(lambda: self._brake),
                    self.drivetrain.apply_request(
                        lambda: (
                            self._aim_at
                            .with_velocity_x(
                                curve(-self.driver_controller.getLeftY(), 0.1) * self._max_speed
                            )
                            .with_velocity_y(
                                curve(-self.driver_controller.getLeftX(), 0.1, 2) * self._max_speed
                            )
                            .target_direction(
                                shooter_utils.angle_aim_to_target(
                                    self.drivetrain.get_pose(),
                                    alliance_flip_util.get_alliance(field_constants.Hub.INNER_CENTER_POINT),
                                )
                            )
                        )
                    ),
                    self.drivetrain.is_facing_angle(
                        shooter_utils.angle_aim_to_target(
                            self.drivetrain.get_pose(),
                            alliance_flip_util.get_alliance(field_constants.Hub.INNER_CENTER_POINT),
                        )
                    ),
                ),

                # passing
                commands2.ConditionalCommand(
                    self.drivetrain.apply_request(lambda: self._brake),
                    self.drivetrain.apply_request(
                        lambda: (
                            self._aim_at
                            .with_velocity_x(
                                curve(-self.driver_controller.getLeftY(), 0.1) * self._max_speed
                            )
                            .with_velocity_y(
                                curve(-self.driver_controller.getLeftX(), 0.1, 2) * self._max_speed
                            )
                            .target_direction(
                                shooter_utils.angle_aim_to_target(
                                    self.drivetrain.get_pose(),
                                    shooter_utils.get_pass_setpoint(
                                        self.drivetrain.get_pose()
                                    )
                                )
                            )
                        )
                    ),
                    self.drivetrain.is_facing_angle(
                        shooter_utils.angle_aim_to_target(
                            self.drivetrain.get_pose(),
                            shooter_utils.get_pass_setpoint(
                                self.drivetrain.get_pose()
                            )
                        )
                    ),
                ),
                alliance_flip_util.get_x(self.drivetrain.get_pose().x()) < field_constants.LinesVertical.ALLIANCE_ZONE
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self.driver_controller.x().whileTrue(self.drivetrain.apply_request(lambda: self._brake))

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        # (self.driver_controller.back() & self.driver_controller.y()).whileTrue(
        #     self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        # )
        # (self.driver_controller.back() & self.driver_controller.x()).whileTrue(
        #     self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        # )
        # (self.driver_controller.start() & self.driver_controller.y()).whileTrue(
        #     self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        # )
        # (self.driver_controller.start() & self.driver_controller.x()).whileTrue(
        #     self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        # )

        Trigger(self.driver_controller.getHID().getPOV() == 180).onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> autos.AutoRoutine:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self.auto_selection.getSelected()
