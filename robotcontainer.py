#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from commands2 import ParallelCommandGroup
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
from subsystems import *

from phoenix6 import swerve
from wpilib import DriverStation, SendableChooser, SmartDashboard

import autos
from utils import math_utils

from subsystems import *
from typing import Callable

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(max_speed)

        self.driver_controller = CommandXboxController(0)
        self.operator_controller = CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()
        self.shooter = Shooter()
        self.climber = Climber()
        self.indexer = Indexer()

        self.auto_selection = SendableChooser()
        self.auto_selection.setDefaultOption("Drive Forward", autos.leave)

        SmartDashboard.putData("Auto", self.auto_selection)
        
    def configureButtonBindings(self) -> None:
        """
        button-command mappings for the indexer subsystem
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        math_utils.curve(-self.driver_controller.getLeftY(), 0.1) * max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        math_utils.curve(-self.driver_controller.getLeftX(), 0.1, 2) * max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self.driver_controller.getRightX() * max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        self.shooter.setDefaultCommand(
            SetShooterIdle(self.shooter)
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )
        self._joystickoperator.rightTrigger().whileTrue(
            DeployIntake(self.intake)
        )

        self._joystickoperator.leftTrigger().whileTrue(
            ReverseIntake(self.intake).onlyIf(lambda: self.intake.is_at_angle(constants.deploy_angle))
        )

        self._joystickoperator.leftBumper().onTrue(
            SetPivot(self.intake, constants.initial_angle)
        )
        # X mode
        self.driver_controller.x().whileTrue(self.drivetrain.apply_request(lambda: self._brake))

        # Rezero drivetrain
        Trigger(lambda: self.driver_controller.getHID().getPOV() == 180).onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
        )

        # Aim drivetrain and shooter
        self.driver_controller.rightTrigger().whileTrue(
            ParallelCommandGroup(
                AimDrivetrain(self.drivetrain, self.driver_controller),
                AimShooter(self.shooter, self.drivetrain)
            )
        )

        # force the indexer to spin
        self.operator_controller.a().or_(self.driver_controller.a()).whileTrue(
            RunIndexer(self.indexer)
        )

        # reverse the indexer
        self.operator_controller.y().onTrue(
            RunIndexerReversed(self.indexer)
        )

        # deploy climb
        self.operator_controller.start().onTrue(
            DeployClimbL1(self.climber)
        )
        
        # climb
        self.operator_controller.back().whileTrue(
            Retract(self.climber)
        )

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

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> Callable[[RobotContainer], autos.AutoRoutine]:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Simple drive forward auton
        idle = swerve.requests.Idle()
        return cmd.sequence(
            # Reset our field centric heading to match the robot
            # facing away from our alliance station wall (0 deg).
            self.drivetrain.runOnce(
                lambda: self.drivetrain.seed_field_centric(Rotation2d.fromDegrees(0))
            ),
            # Then slowly drive forward (away from us) for 5 seconds.
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(0.5)
                    .with_velocity_y(0)
                    .with_rotational_rate(0)
                )
            )
            .withTimeout(5.0),
            # Finally idle for the rest of auton
            self.drivetrain.apply_request(lambda: idle)
        )
