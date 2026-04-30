from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
from wpilib import SmartDashboard
from subsystems import *
from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup, RepeatCommand, WaitCommand, WaitUntilCommand



def auto(robot_container: RobotContainer, path_name: str) -> AutoRoutine:
    path = PathPlannerPath.fromChoreoTrajectory(path_name, 0)
    wait_time = SmartDashboard.getNumber("Auto Wait Time", 4.0)
    command = SequentialCommandGroup(
        WaitCommand(wait_time),
        ParallelDeadlineGroup(
            AutoBuilder.followPath(path),
            SequentialCommandGroup(
                WaitCommand(1.5),
                DeployIntake(robot_container.intake).withTimeout(1),
                InstantCommand(lambda: robot_container.intake.slide_motor_left.set_position(intake_deploy_position/slide_couple_ratio)),
                RunIntake(robot_container.intake)
            )
        ),
        InstantCommand(lambda: robot_container.drivetrain.set_control(requests.Idle())),
        WaitCommand(0.3),
        ParallelDeadlineGroup(
            WaitUntilCommand(lambda: robot_container.shooter.ready_to_shoot() and robot_container.drivetrain.ready_to_shoot),
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain),
            AimDrivetrainAuto(robot_container.drivetrain),
        ).withTimeout(1),
        ParallelCommandGroup(
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain),
            AimDrivetrainAuto(robot_container.drivetrain),
            RunIndexer(robot_container.indexer),
            WaitCommand(0.7).andThen(RepeatCommand(SequentialCommandGroup(
                IntakeIndex(robot_container.intake).withTimeout(0.2),
                DeployIntake(robot_container.intake).withTimeout(0.2)
            )))
        ),
    )

    return AutoRoutine(command, path.getStartingHolonomicPose())