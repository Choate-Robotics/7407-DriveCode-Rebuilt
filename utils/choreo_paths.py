from pathplannerlib.path import PathPlannerPath


def load_path(trajectory_name: str, split_index: int) -> PathPlannerPath:
    """
    Loads one split of a Choreo trajectory with its red-alliance flip already made.

    On red, FollowPathCommand flips the path when it starts, which rebuilds every
    trajectory sample in Python - a slow first loop right when the robot should start
    moving. Autos are built in robotInit, so flipping here makes that a lookup instead.
    """
    path = PathPlannerPath.fromChoreoTrajectory(trajectory_name, split_index)
    flipped = path.flipPath()
    path.flipPath = lambda: flipped
    return path


def path_time(path: PathPlannerPath) -> float:
    """Duration of a Choreo path in seconds (the config is only used for non-Choreo paths)."""
    return path.getIdealTrajectory(None).getTotalTimeSeconds()
