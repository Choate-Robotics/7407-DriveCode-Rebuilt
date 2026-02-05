from commands2 import Command
from wpimath.geometry import Pose2d

class AutoRoutine:
    def __init__(self, command: Command, start_pose: Pose2d):
        self.command = command
        self.start_pose = start_pose