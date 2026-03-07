from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
import robot_constants
from subsystems import *
from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup, RepeatCommand, WaitCommand, WaitUntilCommand



path_name = "OneLoopRightClimb"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(2)]
path = PathPlannerPath.fromChoreoTrajectory(path_name)
def auto(robot_container: RobotContainer) -> AutoRoutine:
    command = SequentialCommandGroup(
        ParallelDeadlineGroup(
            AutoBuilder.followPath(paths[0]),
            DeployIntake(robot_container.intake).andThen(RunIntake(robot_container.intake))
        ),
        InstantCommand(robot_container.drivetrain.set_control(requests.Idle())),
        WaitCommand(0.3),
        ParallelDeadlineGroup(
            WaitUntilCommand(lambda: robot_container.shooter.ready_to_shoot() and robot_container.drivetrain.ready_to_shoot),
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain),
            AimDrivetrainAuto(robot_container.drivetrain),
        ),
        ParallelCommandGroup(
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain),
            AimDrivetrainAuto(robot_container.drivetrain),
            DeployClimbL1(robot_container.climber),
            RunIndexer(robot_container.indexer),
            WaitCommand(1.5).andThen(RepeatCommand(
                IntakeIndex(robot_container.intake).withTimeout(0.5).andThen(DeployIntake(robot_container.intake))   
            ))
        ).withTimeout(5),
        
        AutoBuilder.followPath(paths[1]).deadlineWith(DeployIntake(robot_container.intake)),

        RetractClimb(robot_container.climber)
    )  
        # AutoBuilder.followPath(path),
        # AutoBuilder.followPath(paths[0]),
        # AutoBuilder.followPath(paths[1]),

    return AutoRoutine(command, paths[0].getStartingHolonomicPose())