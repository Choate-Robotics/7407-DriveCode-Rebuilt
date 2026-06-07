#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from commands2 import ParallelCommandGroup, SequentialCommandGroup, SelectCommand, InstantCommand
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

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

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
        # self.left_cam = PhotonCamCustom(left_cam_name, left_cam_transform)
        # self.right_cam = PhotonCamCustom(right_cam_name, right_cam_transform)
        # self.back_cam = PhotonCamCustom(back_cam_name, back_cam_transform)
        self.front_cam = PhotonCamCustom(front_cam_name, front_cam_transform)
        cams = [
            # self.left_cam,
            # self.right_cam,
            # self.back_cam,
            self.front_cam
        ]

        self.backup_gyro = hardware.Pigeon2(26, "canivore")

        self.field_odometry = FieldOdometry(self.drivetrain, cams, self.backup_gyro)
        
        SmartDashboard.setDefaultNumber("Auto Wait Time", 4.0)

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

    def telemetrize_drivetrain(self):
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )
        
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
                        math_utils.curve(-self.driver_controller.getLeftY(), deadband) * max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        math_utils.curve(-self.driver_controller.getLeftX(), deadband) * max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        math_utils.curve(-self.driver_controller.getRightX(), deadband, curve) * max_angular_rate
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

        # only aim shooter
        self.driver_controller.leftTrigger().whileTrue(
            SelectCommand(
                {
                    0: SetShooter(self.shooter, hub_flywheel_velocity, hub_hood_angle),           
                    90: SetShooter(self.shooter, rightpass_flywheel_velocity, rightpass_hood_angle),         
                    180: SetShooter(self.shooter, tower_flywheel_velocity, tower_hood_angle),
                    270: SetShooter(self.shooter, tower_flywheel_velocity, hub_hood_angle),
                    -1: AimShooter(self.shooter, self.drivetrain),
                },
                self.operator_controller.getHID().getPOV
            )
        )

        # command used to tune the shooter by taking in a value from networktables
        self.driver_controller.leftBumper().whileTrue(
            ParallelCommandGroup(
                TuneShooter(self.shooter, self.drivetrain),
                AimDrivetrain(self.drivetrain, self.driver_controller)
            )
        )

        # automatically index
        Trigger(lambda: self.drivetrain.ready_to_shoot and self.shooter.ready_to_shoot()).and_(lambda: DriverStation.isTeleop()).debounce(0.2, Debouncer.DebounceType.kFalling).whileTrue(
            RunIndexer(self.indexer)
        )

        # drive in "snake mode" (intake faces direction of travel)
        self.driver_controller.rightBumper().whileTrue(
            SnakeMode(self.drivetrain, self.driver_controller)
        )

        # force the indexer to spin
        self.operator_controller.a().or_(self.driver_controller.a()).whileTrue(
            # Index(self.indexer, self.intake)
            RunIndexer(self.indexer)
        )

        # reverse the indexer
        self.operator_controller.y().or_(self.driver_controller.y()).whileTrue(
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