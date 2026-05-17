from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
from wpilib import SmartDashboard
from subsystems import *
from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup, RepeatCommand, WaitCommand, WaitUntilCommand



def auto(robot_container: RobotContainer, path_name: str) -> AutoRoutine:
    path = PathPlannerPath.fromChoreoTrajectory(path_name, 0)
    wait_time = SmartDashboard.getNumber("Auto Wait Time", 4.0)
    command = SequentialCommandGroup(
        WaitCommand(wait_time),
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