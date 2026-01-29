from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from phoenix6.swerve import requests

from robotcontainer import RobotContainer

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, InstantCommand

path_name = "Leave"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(1)]

command = SequentialCommandGroup(
    AutoBuilder.followPath(paths[0]),
    RobotContainer.drivetrain.apply_request(lambda: requests.Idle())
)

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())