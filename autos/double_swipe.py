from utils.choreo_paths import load_path, path_time
from pathplannerlib.auto import AutoBuilder
from phoenix6.swerve import requests
from autos import AutoRoutine
from robotcontainer import RobotContainer
import robot_constants
from subsystems import *
from subsystems.indexer.constants import hopper_min_shoot_time
from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, ParallelDeadlineGroup, RepeatCommand, WaitCommand, WaitUntilCommand


# Start spinning the shooter up this long before the end of a path that ends in a shot
shooter_spinup_lead = 1.0
# Wait after the start before deploying the intake. The robot is then at the same spot
# past the trench (x ~6.65 m) as with the old 1 s wait on the slower launch.
intake_deploy_delay = 0.9

def auto(robot_container: RobotContainer, path_name: str) -> AutoRoutine:
    paths = [load_path(path_name, i) for i in range(2)]
    command = SequentialCommandGroup(
        ParallelDeadlineGroup(
            AutoBuilder.followPath(paths[0]),
            SequentialCommandGroup(
                WaitCommand(intake_deploy_delay),
                DeployIntake(robot_container.intake).withTimeout(1),
                InstantCommand(lambda: robot_container.intake.slide_motor_left.set_position(intake_deploy_position/slide_couple_ratio)),
                RunIntake(robot_container.intake)
            ),
            # Spin the shooter up over the end of the path so it's ready on arrival
            WaitCommand(max(path_time(paths[0]) - shooter_spinup_lead, 0)).andThen(
                SetShooterAuto(robot_container.shooter, robot_container.drivetrain)
            )
        ),
        InstantCommand(lambda: robot_container.drivetrain.set_control(requests.Idle())),
        WaitCommand(0.1),
        ParallelDeadlineGroup(
            WaitUntilCommand(lambda: robot_container.shooter.ready_to_shoot() and robot_container.drivetrain.ready_to_shoot),
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain),
            AimDrivetrainAuto(robot_container.drivetrain),
        ).withTimeout(1),
        # Shoot until the hopper reads empty (never for less than hopper_min_shoot_time);
        # the timeout is the backstop if it never reads empty.
        ParallelDeadlineGroup(
            SequentialCommandGroup(
                WaitCommand(hopper_min_shoot_time),
                WaitUntilCommand(robot_container.indexer.hopper_empty),
            ),
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain),
            AimDrivetrainAuto(robot_container.drivetrain),
            RunIndexer(robot_container.indexer),
            # Agitate from the start: the hopper bridges for 1.5-2 s when the slide is still
            RepeatCommand(SequentialCommandGroup(
                IntakeIndex(robot_container.intake).withTimeout(0.3),
                DeployIntake(robot_container.intake).withTimeout(0.2)
            ))
        ).withTimeout(6.0),

        SetShooterIdle(robot_container.shooter).withTimeout(0.2),
        
        ParallelDeadlineGroup(
            AutoBuilder.followPath(paths[1]),
            DeployIntake(robot_container.intake).andThen(RunIntake(robot_container.intake)),
            WaitCommand(max(path_time(paths[1]) - shooter_spinup_lead, 0)).andThen(
                SetShooterAuto(robot_container.shooter, robot_container.drivetrain)
            )
        ),
        InstantCommand(lambda: robot_container.drivetrain.set_control(requests.Idle())),
        WaitCommand(0.1),
        ParallelDeadlineGroup(
            WaitUntilCommand(lambda: robot_container.shooter.ready_to_shoot() and robot_container.drivetrain.ready_to_shoot),
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain),
            AimDrivetrainAuto(robot_container.drivetrain),
        ).withTimeout(1),
        ParallelCommandGroup(
            SetShooterAuto(robot_container.shooter, robot_container.drivetrain),
            AimDrivetrainAuto(robot_container.drivetrain),
            RunIndexer(robot_container.indexer),
            WaitCommand(0.75).andThen(RepeatCommand(SequentialCommandGroup(
                IntakeIndex(robot_container.intake).withTimeout(0.3),
                DeployIntake(robot_container.intake).withTimeout(0.2)
            )))
        ),
    )

    return AutoRoutine(command, paths[0].getStartingHolonomicPose())