from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
import robot_constants
from subsystems import *

from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup

path_name = "TwoLoopClimb"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(7)]
def auto(robot_container: RobotContainer) -> AutoRoutine:
    command = SequentialCommandGroup(

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[0]),
            SetPivot(robot_container.intake, 90)
            )

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[1]),
            RunIntake(robot_container.intake)
        ),

        AutoBuilder.followPath(paths[2]),

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[3]),
            RunIndexer(robot_container.indexer) #replace with index deadline command
        ),

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[4]),
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain)
        ),
        
        ParallelCommandGroup(
            AutoBuilder.followPath(paths[5]),
            SetPivot(robot_container.intake, 90)
            ),

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[6]),
            RunIntake(robot_container.intake)
        ),

        AutoBuilder.followPath(paths[7]),

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[8]),
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain)
        ),

        ParallelCommandGroup(
            AutoBuilder.followPath(paths[9]),
        # index deadlinecommand (the one that retracts intake) added once merge occurs

        ),
        
        ParallelCommandGroup(
            AutoBuilder.followPath(paths[10]),
            DeployClimbL1(robot_container.climber)
        ),
        Retract(robot_container.climber)
    )
    return AutoRoutine(command, paths[0].getStartingHolonomicPose())