"""
Driver buttons that change with driver-only mode are bound once each, with the mode
picked at press time. These check each mode runs the right command, and that a branch
only takes the subsystems it uses (a ConditionalCommand would otherwise require both
branches' subsystems and interrupt unrelated commands).
"""

import commands2
from wpilib import SmartDashboard
from wpilib.simulation import XboxControllerSim

MODE_KEY = "Controls/Driver Only Mode"


def test_driver_only_bindings(control, robot):
    driver = XboxControllerSim(0)
    operator = XboxControllerSim(1)
    scheduler = commands2.CommandScheduler.getInstance()

    def step(seconds=0.2):
        driver.notifyNewData()
        operator.notifyNewData()
        control.step_timing(seconds=seconds, autonomous=False, enabled=True)

    def running(subsystem) -> str:
        cmd = scheduler.requiring(subsystem)
        return cmd.getName() if cmd is not None else ""

    with control.run_robot():
        rc = robot.robot
        default_drive = rc.drivetrain.getDefaultCommand()
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        operator.setPOV(-1)

        # ---- normal mode ----
        SmartDashboard.putBoolean(MODE_KEY, False)
        step()

        # static shot runs on the shooter and leaves the operator's intake alone
        operator.setRightTriggerAxis(1.0)  # operator deploys + runs intake
        step()
        intake_cmd = running(rc.intake)
        assert intake_cmd, "operator intake should be running"
        driver.setLeftBumperButton(True)
        step()
        assert running(rc.shooter) == "SelectCommand", running(rc.shooter)  # static shot (AimShooter inside)
        assert running(rc.intake) == intake_cmd, "static shot must not interrupt the intake"
        driver.setLeftBumperButton(False)
        operator.setRightTriggerAxis(0.0)
        step()
        assert running(rc.shooter) == "SetShooterIdle", "shooter returns to idle on release"

        # snake mode takes the drivetrain while held, default drive comes back on release
        driver.setRightBumperButton(True)
        step()
        assert running(rc.drivetrain) == "SnakeMode", running(rc.drivetrain)
        driver.setRightBumperButton(False)
        step()
        assert scheduler.requiring(rc.drivetrain) is default_drive

        # right stick does nothing to the indexer in normal mode
        driver.setRightStickButton(True)
        step()
        assert running(rc.indexer) != "RunIndexerReversed"
        driver.setRightStickButton(False)
        step()

        # ---- driver-only mode ----
        SmartDashboard.putBoolean(MODE_KEY, True)
        step()

        # left bumper: intake indexing while held, intake stopped on release
        driver.setLeftBumperButton(True)
        step()
        assert running(rc.intake) == "SequentialCommandGroup", running(rc.intake)
        assert running(rc.shooter) == "SetShooterIdle", "no static shot in driver-only mode"
        driver.setLeftBumperButton(False)
        step()
        assert running(rc.intake) == "DeployIntake", running(rc.intake)

        # right bumper: a quick tap still retracts the intake fully, and driving continues
        driver.setRightBumperButton(True)
        step(0.04)
        assert scheduler.requiring(rc.drivetrain) is default_drive, "driver-only right bumper must not stop driving"
        driver.setRightBumperButton(False)
        step(0.04)
        assert running(rc.intake) == "RetractIntake", "retract keeps running after release"

        # right stick: indexer reversed while held
        driver.setRightStickButton(True)
        step()
        assert running(rc.indexer) == "RunIndexerReversed", running(rc.indexer)
        driver.setRightStickButton(False)
        step()
        assert running(rc.indexer) != "RunIndexerReversed"

        # start/back/left stick run without errors
        for button in (driver.setStartButton, driver.setBackButton, driver.setLeftStickButton):
            button(True)
            step(0.1)
            button(False)
            step(0.1)

        SmartDashboard.putBoolean(MODE_KEY, False)
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
