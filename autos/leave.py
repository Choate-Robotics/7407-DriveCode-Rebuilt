from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer

from commands2 import SequentialCommandGroup, InstantCommand

path_name = "Leave"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(1)]

def auto(robot_container: RobotContainer) -> AutoRoutine:
    command = SequentialCommandGroup(
        AutoBuilder.followPath(paths[0]),
        robot_container.drivetrain.apply_request(lambda: requests.Idle()),
    )
    return AutoRoutine(command, paths[0].getStartingHolonomicPose())