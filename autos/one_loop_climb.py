from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
import robot_constants
from subsystems import *
from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup



path_name = "OneLoopClimb"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(1)]
def auto(robot_container: RobotContainer) -> AutoRoutine:
    command = SequentialCommandGroup(
        ParallelCommandGroup(
            AutoBuilder.followPath(paths[0]),
            DeployIntake(robot_container.intake)
        ),

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[1]),
            RunIntake(robot_container.intake)
        ),

        AutoBuilder.followPath(paths[2]),
        ParallelCommandGroup(
            AutoBuilder.followPath(paths[3]),
            SetPivot(robot_container.intake, 90.0)
        ),

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[4]),
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain)
        ),
        RunIndexer(robot_container.indexer).withTimeout(robot_constants.auto_shooting_timeout),
        AutoBuilder.followPath(paths[5]),
        DeployClimbL1(robot_container.climber)
    )
    return AutoRoutine(command, paths[0].getStartingHolonomicPose())