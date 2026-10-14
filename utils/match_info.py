"""Match / hub-shift info for Elastic, published under Match/.

Runs in a background thread at 10 Hz, so it costs the robot loop nothing beyond the
odd ~100 us of interpreter time-sharing.

    Match/Hub Active    our hub is active right now (false while unknown)
    Match/Won Auto      we scored more fuel in auto (false until game data arrives)
    Match/Shift         Auto, Transition, Shift 1-4, End Game, Disabled
    Match/Match Time    seconds left in the current period (auto or teleop)
    Match/Shift Time    seconds left in the current shift
    Match/Game Data     raw FMS game-specific message (empty = not received yet)

Game data: FMS sends 'R' or 'B' shortly after auto, the alliance whose hub is
INACTIVE in shift 1 - the alliance that scored more fuel in auto (FMS picks at
random on a tie). Shifts then alternate, so that alliance is inactive in shifts
1 and 3 and active in 2 and 4. Both hubs are active in auto, the transition shift
and end game.
"""

import threading
import time

from wpilib import DriverStation, SmartDashboard

# Teleop timeline as match time remaining (s), (name, ends at). Check against the
# game manual if the timeline changes.
TELEOP_PHASES = [
    ("Transition", 130.0),
    ("Shift 1", 105.0),
    ("Shift 2", 80.0),
    ("Shift 3", 55.0),
    ("Shift 4", 30.0),
    ("End Game", 0.0),
]


def _game_data_alliance():
    msg = DriverStation.getGameSpecificMessage()
    if msg[:1] == "R":
        return DriverStation.Alliance.kRed
    if msg[:1] == "B":
        return DriverStation.Alliance.kBlue
    return None


def _hub_active(phase: str, inactive_first, ours) -> bool | None:
    """Whether our hub is active during `phase`; None if it can't be known yet."""
    if phase in ("Disabled", "No match time"):
        return None
    if not phase.startswith("Shift "):
        return True  # auto, transition and end game: both hubs active
    if inactive_first is None or ours is None:
        return None
    odd_shift = int(phase[-1]) % 2 == 1
    return (ours == inactive_first) != odd_shift


def _update():
    t = DriverStation.getMatchTime()
    inactive_first = _game_data_alliance()
    ours = DriverStation.getAlliance()

    phase, shift_time = "Disabled", 0.0
    if DriverStation.isAutonomousEnabled():
        phase, shift_time = "Auto", max(t, 0.0)
    elif DriverStation.isTeleopEnabled():
        if t < 0:
            phase = "No match time"  # e.g. enabled without FMS / practice mode
        else:
            for i, (name, ends_at) in enumerate(TELEOP_PHASES):
                if t > ends_at or i == len(TELEOP_PHASES) - 1:
                    phase, shift_time = name, max(t - ends_at, 0.0)
                    break

    SmartDashboard.putBoolean("Match/Hub Active", _hub_active(phase, inactive_first, ours) is True)
    SmartDashboard.putBoolean("Match/Won Auto", ours is not None and inactive_first == ours)
    SmartDashboard.putString("Match/Shift", phase)
    SmartDashboard.putNumber("Match/Match Time", max(t, 0.0))
    SmartDashboard.putNumber("Match/Shift Time", round(shift_time, 1))
    SmartDashboard.putString("Match/Game Data", DriverStation.getGameSpecificMessage())


def start_match_info(period_s: float = 0.1):
    """Start publishing Match/ entries from a background thread. Call once from robotInit."""
    def run():
        while True:
            _update()
            time.sleep(period_s)

    threading.Thread(target=run, name="match-info", daemon=True).start()
