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
            
        )
    )
    return AutoRoutine(command, paths[0].getStartingHolonomicPose())