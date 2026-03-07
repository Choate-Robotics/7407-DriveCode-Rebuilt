from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
import robot_constants
from subsystems import *

from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup

path_name = "slowloop"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(5)]
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

    AutoBuilder.followPath(paths[3]),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[4]),
        AimShooter(robot_container.shooter, robot_container.drivetrain)
    ),

    Index(robot_container.indexer, robot_container.intake),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[5]),
        DeployClimbL1(robot_container.climber)
    ),

    RetractClimb(robot_container.climber)
    )
    return AutoRoutine(command, paths[0].getStartingHolonomicPose())
