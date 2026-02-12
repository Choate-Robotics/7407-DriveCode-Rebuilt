from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
from subsystems import *

from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup

path_name = "Depot"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(11)]

def auto(robot_container: RobotContainer) -> AutoRoutine:
    command = SequentialCommandGroup(
        AutoBuilder.followPath(paths[0]),
        DeployIntake(robot_container.intake),
        RunIndexer(robot_container.indexer),
        AutoBuilder.followPath(paths[1]),
        AimDrivetrainAuto(robot_container.drivetrain),
        SetShooterAuto(robot_container.shooter),
        AutoBuilder.followPath(paths[2]),
        DeployClimbL1(robot_container.climber),
        robot_container.drivetrain.apply_request(lambda: requests.Idle())
    )
    return AutoRoutine(command, paths[0].getStartingHolonomicPose())
