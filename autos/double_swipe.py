from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
import robot_constants
from subsystems import *
from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup, RepeatCommand, WaitCommand, WaitUntilCommand



def auto(robot_container: RobotContainer, path_name: str) -> AutoRoutine:
    paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(2)]
    command = SequentialCommandGroup(
        ParallelDeadlineGroup(
            AutoBuilder.followPath(paths[0]),
            WaitCommand(0.75).andThen(DeployIntake(robot_container.intake).andThen(RunIntake(robot_container.intake)))
        ),
        InstantCommand(robot_container.drivetrain.set_control(requests.Idle())),
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
            WaitCommand(2.25),
            IntakeIndex(robot_container.intake)
        ).withTimeout(4),

        SetShooterIdle(robot_container.shooter).withTimeout(0.2),
        
        ParallelDeadlineGroup(
            AutoBuilder.followPath(paths[1]),
            DeployIntake(robot_container.intake).andThen(RunIntake(robot_container.intake))
        ),
        InstantCommand(robot_container.drivetrain.set_control(requests.Idle())),
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
            WaitCommand(1.75),
            IntakeIndex(robot_container.intake)
        ).withTimeout(5),
    )  
        # AutoBuilder.followPath(path),
        # AutoBuilder.followPath(paths[0]),
        # AutoBuilder.followPath(paths[1]),

    return AutoRoutine(command, paths[0].getStartingHolonomicPose())