import commands2
from .constants import *
from .subsystem import Climber
from utils import local_logger

logger = local_logger.LocalLogger("ClimberCommands")

class DeployClimbL1(commands2.Command):
    """
    deploy climber to L1 position.
    uses set_position method from Climber subsystem.
    checks if both of the motors positions is at or above L1 position to finish.
    """

    def __init__(self, subsystem: Climber):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self):
        pass

    def execute(self):
        self.subsystem.set_voltage(climber_extend_voltage)

    def isFinished(self):
        return self.subsystem.get_motor_position() >= L1_pos
    
    def end(self, interrupted: bool):
        self.subsystem.moving = False


class RetractClimb(commands2.Command):
    """
    lower climber to climb robot.
    uses set_voltage method from Climber subsystem.
    checks if both of the motors positions is at lower bound to finish.
    """

    def __init__(self, subsystem: Climber):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)   

    def initialize(self):
        self.subsystem.set_voltage(climber_retract_voltage)

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.is_motor_position(climber_lower_bound)

    def end(self, interrupted: bool):
        self.subsystem.moving = False
        self.subsystem.set_voltage(0)