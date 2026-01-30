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
        print(f"\n{'='*20}\nDEBUG: {"auto complete:"}\n{'='*20}\n"),
        robot_container.drivetrain.apply_request(lambda: requests.Idle()),
        print(f"\n{'='*20}\nDEBUG: {"idling"}\n{'='*20}\n"),
    )
    return AutoRoutine(command, paths[0].getStartingHolonomicPose())