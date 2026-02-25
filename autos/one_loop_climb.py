from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
import robot_constants
from subsystems import *
from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup



path_name = "OneLoopClimb"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(8)]
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

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[2]),
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain.get_pose())
        ),

        Index(robot_container.indexer, robot_container.intake).withTimeout(5),
        
        ParallelCommandGroup(
            AutoBuilder.followPath(paths[3]),
            DeployClimbL1(robot_container.climber)
        ),

        RetractClimb(robot_container.climber)
    )  
    return AutoRoutine(command, paths[0].getStartingHolonomicPose()) 