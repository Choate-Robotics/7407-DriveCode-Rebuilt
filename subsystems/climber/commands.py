from commands2 import SequentialCommandGroup
import constants
import constants
from subsystems.climber.climber import Climber
from utils import local_logger

logger = local_logger.LocalLogger("ClimberCommands")


class DeployClimbL1(SequentialCommandGroup):

    def __init__(self, subsystem: Climber):
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.set_position(constants.climber_L1)

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.get_left_motor_position() >= constants.climber_L1 and self.subsystem.get_right_motor_position() >= constants.climber_L1

    def end(self, interrupted: bool):
        self.subsystem.moving = False


class DeployClimbL3(SequentialCommandGroup):

    def __init__(self, subsystem: Climber):
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.set_position(constants.climber_L3)

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.get_left_motor_position() >= constants.climber_L3 and self.subsystem.get_right_motor_position() >= constants.climber_L3

    def end(self, interrupted: bool):
            self.subsystem.moving = False


class Climb(SequentialCommandGroup):

    def __init__(self, subsystem: Climber):
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.set_voltage(constants.climber_drop_voltage)

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.get_left_motor_position() <= constants.climber_lower_bound and self.subsystem.get_right_motor_position() <= constants.climber_lower_bound

    def end(self, interrupted: bool):
        self.subsystem.moving = False
