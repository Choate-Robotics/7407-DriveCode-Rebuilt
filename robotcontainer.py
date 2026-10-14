#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from commands2 import ParallelCommandGroup, SequentialCommandGroup, SelectCommand, InstantCommand, ConditionalCommand, ScheduleCommand
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
from subsystems import *
from sensors import *
from robot_constants import *

from phoenix6 import swerve, hardware
from wpilib import DriverStation, SendableChooser, SmartDashboard
from wpimath.filter import Debouncer

import autos
from utils import math_utils
from typing import Callable

from utils import alliance_flip_util
from utils.device_health import DeviceHealth, find_devices

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    _DRIVER_ONLY_MODE_KEY = "Controls/Driver Only Mode"

    def __init__(self) -> None:

        # Initialize drive requests
        self._drive = (
            swerve.requests.FieldCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()

        self._logger = Telemetry(max_speed)

        # Initialize controllers

        self.driver_controller = CommandXboxController(0)
        self.operator_controller = CommandXboxController(1)

        # Initialize subsystems

        self.drivetrain = TunerConstants.create_drivetrain()
        self.shooter = Shooter()
        # self.climber = Climber()
        self.indexer = Indexer()
        self.intake = Intake()

        # Initialize odometry
        # front_cam and front_left_cam run on the coprocessor (see the vision repo),
        # which publishes both cameras' pose observations as one batched NT4 struct-array
        # topic - one read per loop covers both cameras, instead of polling one PhotonCamera
        # per loop in rotation.
        self.vision = BatchedVisionSource()

        self.backup_gyro = hardware.Pigeon2(26, "canivore")

        self.field_odometry = FieldOdometry(self.drivetrain, self.vision, self.backup_gyro,
                                            speeds_supplier=lambda: self._logger.latest_speeds)
        
        SmartDashboard.setDefaultNumber("Auto Wait Time", 4.0)

        # Not persistent - always boots up in normal (two-driver) control mode.
        SmartDashboard.setDefaultBoolean(self._DRIVER_ONLY_MODE_KEY, False)
        self._driver_only = False

        # Initialize auto chooser
        self.auto_selection = SendableChooser()
        self.auto_selection.setDefaultOption("Drive Forward", autos.leave(self))
        self.auto_selection.addOption("Double swipe left greedy", autos.double_swipe(self, "DoubleSwipeLeftGreedy"))
        self.auto_selection.addOption("Double swipe right greedy", autos.double_swipe(self, "DoubleSwipeRightGreedy"))
        self.auto_selection.addOption("Double swipe left shallow", autos.double_swipe(self, "DoubleSwipeLeftShallow"))
        self.auto_selection.addOption("Double swipe right shallow", autos.double_swipe(self, "DoubleSwipeRightShallow"))
        self.auto_selection.addOption("Double swipe right BC dot", autos.double_swipe_battlecry(self, "DoubleSwipeRightShallowDot"))
        self.auto_selection.addOption("Double swipe left BC dot", autos.double_swipe_battlecry(self, "DoubleSwipeLeftShallowDot"))
        self.auto_selection.addOption("Follow left greedy", autos.follow(self, "SingleSwipeLeftGreedy"))
        self.auto_selection.addOption("Follow left depot", autos.follow(self, "SingleSwipeDepot"))
        self.auto_selection.addOption("Follow right greedy", autos.follow(self, "SingleSwipeRightGreedy"))
        self.auto_selection.addOption("Depot", autos.depot(self))
        self.auto_selection.addOption("Straight line LEFT", autos.straight_line(self, "StraightLine"))
        self.auto_selection.addOption("Straight line RIGHT", autos.straight_line(self, "RIGHTSTRAIGHTLINE"))
        # self.auto_selection.addOption("COMP depot neutral", autos.comp_depot(self))

        SmartDashboard.putData("Auto", self.auto_selection)

        # Health/ on the dashboard + Elastic pop-ups when a device disconnects
        self.device_health = DeviceHealth(find_devices(self), self.vision)

    def telemetrize_drivetrain(self):
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def refresh_driver_only_mode(self) -> None:
        """Read the switch once per loop (from robotPeriodic) instead of in every trigger."""
        self._driver_only = SmartDashboard.getBoolean(self._DRIVER_ONLY_MODE_KEY, False)

    def _drive_request(self):
        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        vx, vy = math_utils.radial_curve(
            -self.driver_controller.getLeftY(),
            -self.driver_controller.getLeftX(),
            translation_deadband_min,
            translation_deadband_max,
        )
        return (
            self._drive.with_velocity_x(vx * max_speed)  # Drive forward with negative Y (forward)
            .with_velocity_y(vy * max_speed)  # Drive left with negative X (left)
            .with_rotational_rate(
                math_utils.curve(-self.driver_controller.getRightX(), deadband, curve) * max_angular_rate
            )  # Drive counterclockwise with negative X (left)
        )

    def _driver_only_mode(self) -> bool:
        return self._driver_only

    def configureButtonBindings(self) -> None:
        """
        button-command mappings for the indexer subsystem
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(self._drive_request)
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

        # X mode
        self.driver_controller.x().whileTrue(self.drivetrain.apply_request(lambda: self._brake))

        # Rezero drivetrain
        self.driver_controller.povDown().onTrue(
            ParallelCommandGroup(
                self.drivetrain.runOnce(self.drivetrain.seed_field_centric),
                InstantCommand(lambda: self.backup_gyro.set_yaw(alliance_flip_util.get_alliance(Rotation2d()).degrees()))
            )
        )       

        # aim drivetrain and shooter based on operator input
        self.driver_controller.rightTrigger().whileTrue(
            SelectCommand(
                {
                    0: ParallelCommandGroup(
                        DriveAtAngle(self.drivetrain, self.driver_controller, hub_drivetrain_angle),
                        SetShooter(self.shooter, hub_flywheel_velocity, hub_hood_angle)            
                    ),
                    90: ParallelCommandGroup(
                        DriveAtAngle(self.drivetrain, self.driver_controller, rightpass_drivetrain_angle),
                        SetShooter(self.shooter, rightpass_flywheel_velocity, rightpass_hood_angle)            
                    ),
                    180: ParallelCommandGroup(
                        DriveAtAngle(self.drivetrain, self.driver_controller, tower_drivetrain_angle),
                        SetShooter(self.shooter, tower_flywheel_velocity, tower_hood_angle)
                    ),
                    270: ParallelCommandGroup(
                        DriveAtAngle(self.drivetrain, self.driver_controller, hub_drivetrain_angle),
                        SetShooter(self.shooter, tower_flywheel_velocity, hub_hood_angle)
                    ),
                    -1: ParallelCommandGroup(
                        AimShooter(self.shooter, self.drivetrain),
                        AimDrivetrain(self.drivetrain, self.driver_controller)
                    )
                },
                self.operator_controller.getHID().getPOV
            )
        )

        # deploy and run intake (both modes; mirrors operator's right trigger)
        self.driver_controller.leftTrigger().whileTrue(
            SequentialCommandGroup(
                DeployIntake(self.intake),
                RunIntake(self.intake)
            )
        )

        # Driver buttons that do different things in driver-only mode are bound once, with
        # the mode picked at press time (flipping the mode mid-press takes effect on the
        # next press). Held branches run as proxies so each only requires its own
        # subsystems (a ConditionalCommand requires both branches'); one-shot branches
        # that must finish after release use ScheduleCommand, like the old onTrue/onFalse.

        # left bumper: static shot (only aim shooter) / driver-only: intake indexing
        # (mirrors operator's left bumper)
        self.driver_controller.leftBumper().whileTrue(
            ConditionalCommand(
                IntakeIndex(self.intake).andThen(RunIntake(self.intake, index_speed)).asProxy(),
                SelectCommand(
                    {
                        0: SetShooter(self.shooter, hub_flywheel_velocity, hub_hood_angle),
                        90: SetShooter(self.shooter, rightpass_flywheel_velocity, rightpass_hood_angle),
                        180: SetShooter(self.shooter, tower_flywheel_velocity, tower_hood_angle),
                        270: SetShooter(self.shooter, tower_flywheel_velocity, hub_hood_angle),
                        -1: AimShooter(self.shooter, self.drivetrain),
                    },
                    self.operator_controller.getHID().getPOV
                ).asProxy(),
                self._driver_only_mode,
            )
        ).onFalse(
            ConditionalCommand(ScheduleCommand(DeployIntake(self.intake, speed=0)), InstantCommand(), self._driver_only_mode)
        )

        # command used to tune the shooter by taking in a value from networktables
        # self.driver_controller.leftBumper().whileTrue(
        #     ParallelCommandGroup(
        #         TuneShooter(self.shooter, self.drivetrain),
        #         AimDrivetrain(self.drivetrain, self.driver_controller)
        #     )
        # )

        # automatically index
        Trigger(lambda: self.drivetrain.ready_to_shoot and self.shooter.ready_to_shoot()).and_(lambda: DriverStation.isTeleop()).debounce(0.2, Debouncer.DebounceType.kFalling).whileTrue(
            RunIndexer(self.indexer)
        )

        # right bumper: snake mode (intake faces direction of travel) / driver-only:
        # retract intake (mirrors operator's right bumper; runs to completion like onTrue)
        self.driver_controller.rightBumper().whileTrue(
            ConditionalCommand(
                ScheduleCommand(RetractIntake(self.intake)),
                SnakeMode(self.drivetrain, self.driver_controller).asProxy(),
                self._driver_only_mode,
            )
        )

        # driver-only mode: zero intake (mirrors operator's L3)
        self.driver_controller.leftStick().onTrue(
            ConditionalCommand(InstantCommand(lambda: self.intake.zero_intake()), InstantCommand(), self._driver_only_mode)
        )

        # driver-only mode: run indexer backward (mirrors operator's R3)
        self.driver_controller.rightStick().whileTrue(
            ConditionalCommand(RunIndexerReversed(self.indexer).asProxy(), InstantCommand(), self._driver_only_mode)
        )

        # driver-only mode: manual intake in/out (mirrors operator's start/back)
        self.driver_controller.start().onTrue(
            ConditionalCommand(InstantCommand(lambda: self.intake.set_slide_motor_voltage(-3)), InstantCommand(), self._driver_only_mode)
        ).onFalse(
            ConditionalCommand(InstantCommand(lambda: self.intake.stop_pivot()), InstantCommand(), self._driver_only_mode)
        )

        self.driver_controller.back().onTrue(
            ConditionalCommand(InstantCommand(lambda: self.intake.set_slide_motor_voltage(3)), InstantCommand(), self._driver_only_mode)
        ).onFalse(
            ConditionalCommand(InstantCommand(lambda: self.intake.stop_pivot()), InstantCommand(), self._driver_only_mode)
        )

        # force the indexer to spin
        self.driver_controller.a().whileTrue(
            # Index(self.indexer, self.intake)
            RunIndexer(self.indexer)
        )

        # reverse the indexer
        self.driver_controller.y().whileTrue(
            ClearTower(self.indexer, self.shooter)
        )
        
        # deploy and run intake
        self.operator_controller.rightTrigger().whileTrue(
            SequentialCommandGroup(
                DeployIntake(self.intake),
                RunIntake(self.intake)
            )
        )

        # run intake in reverse
        self.operator_controller.leftTrigger().whileTrue(
            ReverseIntake(self.intake)
        )

        self.operator_controller.rightBumper().onTrue(
            RetractIntake(self.intake)
        )

        self.operator_controller.start().onTrue(
            InstantCommand(lambda: self.intake.set_slide_motor_voltage(-3))
        ).onFalse(InstantCommand(lambda: self.intake.stop_pivot()))

        self.operator_controller.back().onTrue(
            InstantCommand(lambda: self.intake.set_slide_motor_voltage(3))
        ).onFalse(InstantCommand(lambda: self.intake.stop_pivot()))

        self.operator_controller.leftStick().onTrue(
            InstantCommand(lambda: self.intake.zero_intake())
        )

        self.operator_controller.rightStick().or_(self.driver_controller.b()).whileTrue(
            RunIndexerReversed(self.indexer)
        )

        self.operator_controller.leftBumper().onTrue(
            IntakeIndex(self.intake).andThen(RunIntake(self.intake, index_speed))
        ).onFalse(DeployIntake(self.intake, speed=0))

        # trim flywheel

        self.operator_controller.x().onTrue(
            InstantCommand(lambda: self.shooter.trim_down())
        )

        self.operator_controller.b().onTrue(
            InstantCommand(lambda: self.shooter.trim_up())
        )

        # trim hood (Y up, A down; mirrors X/B for the flywheel)

        self.operator_controller.a().onTrue(
            InstantCommand(lambda: self.shooter.trim_hood_down())
        )

        self.operator_controller.y().onTrue(
            InstantCommand(lambda: self.shooter.trim_hood_up())
        )

        # deploy climb
        # self.operator_controller.start().onTrue(
        #     DeployClimbL1(self.climber)
        # )
        
        # climb
        # self.operator_controller.back().whileTrue(
        #     RetractClimb(self.climber)
        # )

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

    def getAutonomousCommand(self) -> autos.AutoRoutine:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self.auto_selection.getSelected()