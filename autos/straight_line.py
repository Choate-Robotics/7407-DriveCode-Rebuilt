from utils.choreo_paths import load_path
from pathplannerlib.auto import AutoBuilder
from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
from wpilib import SmartDashboard
from subsystems import *
from commands2 import DeferredCommand, SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup, RepeatCommand, WaitCommand, WaitUntilCommand



def auto(robot_container: RobotContainer, path_name: str) -> AutoRoutine:
    path = load_path(path_name, 0)
    command = SequentialCommandGroup(
        # Read when auto starts, so the dashboard value at match time is the one used
        DeferredCommand(lambda: WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 4.0))),
        ParallelDeadlineGroup(
            AutoBuilder.followPath(path),
            SequentialCommandGroup(
                DeployIntake(robot_container.intake).withTimeout(1),
                InstantCommand(lambda: robot_container.intake.slide_motor_left.set_position(intake_deploy_position/slide_couple_ratio)),
                RunIntake(robot_container.intake)
            )
        )
    )

    return AutoRoutine(command, path.getStartingHolonomicPose())