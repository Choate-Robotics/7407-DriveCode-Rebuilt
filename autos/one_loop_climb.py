from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
import robot_constants
from subsystems import *
from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup


path_name = "OneLoopClimb"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(6)]

def auto(robot_container: RobotContainer) -> AutoRoutine:
    command = SequentialCommandGroup(
        ParallelCommandGroup(
            AutoBuilder.followPath(paths[0]),
            SequentialCommandGroup(
            DeployIntake(robot_container.intake),
            RunIntake(robot_container.intake)
            )
        ),
        SequentialCommandGroup(
            AutoBuilder.followPath(paths[1]),
            #stop intake from running?
            AutoBuilder.followPath(paths[2]),

        ),

        SequentialCommandGroup(
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain), 
            RunIndexer(robot_container.indexer).withTimeout(robot_constants.auto_shooting_timeout),
        ),
        ParallelCommandGroup(
        AutoBuilder.followPath(paths[3]),
        DeployClimbL1(robot_container.climber),
        ),
        
        Retract(robot_container.climber)
    )
    return AutoRoutine(command, paths[0].getStartingHolonomicPose())