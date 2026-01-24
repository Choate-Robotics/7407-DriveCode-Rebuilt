from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from phoenix6.swerve import requests

from robotcontainer import RobotContainer

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, InstantCommand

path_name = "Leave"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(0)]

command = SequentialCommandGroup(
    RobotContainer.drivetrain.apply_request(
        lambda: requests.FieldCentric()
        .with_velocity_x(-0.5)
    ).withTimeout(2.5)
)

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())