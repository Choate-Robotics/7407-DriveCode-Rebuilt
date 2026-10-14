"""
Runs a few autonomous routines in simulation for a full 20 s auto period, on both alliances,
so a crash in an auto (a bad path name, a command group that won't build, an exception
in a command) is caught by `robotpy test` and therefore before every deploy.

Any exception raised in a command fails the test.
"""

import hal
import pytest
from ntcore import NetworkTableInstance
from wpilib import DriverStation
from wpilib.simulation import DriverStationSim

# One of each kind of routine; every extra case is another simulated robot started at once
AUTOS = [
    "Double swipe left greedy",
    "Double swipe right BC dot",
    "Follow left greedy",
]


@pytest.mark.parametrize("alliance", [DriverStation.Alliance.kBlue, DriverStation.Alliance.kRed])
@pytest.mark.parametrize("auto_name", AUTOS)
def test_auto(control, robot, auto_name, alliance):
    with control.run_robot():
        DriverStationSim.setAllianceStationId(
            hal.AllianceStationID.kRed1 if alliance == DriverStation.Alliance.kRed else hal.AllianceStationID.kBlue1
        )
        DriverStationSim.notifyNewData()
        chooser = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Auto")
        chooser.putString("selected", auto_name)
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)
        assert chooser.getString("active", "") == auto_name

        control.step_timing(seconds=20.0, autonomous=True, enabled=True)
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)


def test_auto_twice_then_reselect(control, robot):
    """Enabling the same auto twice without restarting code, then picking another one."""
    with control.run_robot():
        chooser = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Auto")
        chooser.putString("selected", "Double swipe left greedy")
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)
        for _ in range(2):
            control.step_timing(seconds=3.0, autonomous=True, enabled=True)
            control.step_timing(seconds=0.5, autonomous=True, enabled=False)

        chooser.putString("selected", "Depot")
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)
        assert chooser.getString("active", "") == "Depot"
        control.step_timing(seconds=3.0, autonomous=True, enabled=True)
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)
