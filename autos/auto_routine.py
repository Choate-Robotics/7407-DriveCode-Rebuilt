from commands2 import Command
from wpimath.geometry import Pose2d

import math
from utils.field_constants import LinesVertical, LinesHorizontal, get_red_pose
from wpimath.geometry import Rotation2d, Translation2d

class AutoRoutine:
    def __init__(self, command: Command, start_pose: Pose2d):
        self.command = command
        self.blue_start_pose = start_pose
        self.red_start_pose = get_red_pose(start_pose)