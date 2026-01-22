from commands2 import Command
from wpimath.geometry import Pose2d

import math
from utils.field_constants import FieldConstants
from wpimath.geometry import Rotation2d, Translation2d

class AutoRoutine:
    def __init__(self, command: Command, start_pose: Pose2d):
        self.command = command
        self.blue_start_pose = start_pose
        self.red_start_pose = start_pose.rotateAround(Translation2d(FieldConstants.field_length/2, FieldConstants.field_width/2) , Rotation2d(math.pi/2))