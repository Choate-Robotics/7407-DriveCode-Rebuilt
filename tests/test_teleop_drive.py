"""
Drives the robot in simulated teleop with stick and button input, so code that only
runs while enabled (the default drive command, aiming, presets, snake mode, trims) is
exercised by `robotpy test` and therefore before every deploy. The builtin pyfrc test
only runs disabled, which is how a crash in the drive command once got past it.

Any exception raised in a command fails the test.
"""

from wpilib.simulation import XboxControllerSim


def test_teleop_driving(control, robot):
    driver = XboxControllerSim(0)
    operator = XboxControllerSim(1)

    def step(seconds):
        driver.notifyNewData()
        operator.notifyNewData()
        control.step_timing(seconds=seconds, autonomous=False, enabled=True)

    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)

        # default drive command: translate and rotate
        driver.setLeftY(-0.8)
        driver.setLeftX(0.3)
        driver.setRightX(0.4)
        step(1.0)

        # right trigger, no D-pad: auto-aim (AimDrivetrain + AimShooter)
        operator.setPOV(-1)
        driver.setRightTriggerAxis(1.0)
        step(1.0)

        # operator trims while aiming (flywheel X/B, hood A/Y)
        for button in (operator.setBButton, operator.setXButton, operator.setYButton, operator.setAButton):
            button(True)
            step(0.1)
            button(False)
            step(0.1)

        # right trigger with a D-pad preset: DriveAtAngle + SetShooter
        driver.setRightTriggerAxis(0.0)
        step(0.2)
        operator.setPOV(0)
        driver.setRightTriggerAxis(1.0)
        step(1.0)
        driver.setRightTriggerAxis(0.0)
        operator.setPOV(-1)

        # right bumper: snake mode
        driver.setRightBumperButton(True)
        step(1.0)
        driver.setRightBumperButton(False)

        # left bumper: static shot (only aim shooter)
        driver.setLeftBumperButton(True)
        step(0.5)
        driver.setLeftBumperButton(False)

        driver.setLeftY(0.0)
        driver.setLeftX(0.0)
        driver.setRightX(0.0)
        step(0.5)

        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
