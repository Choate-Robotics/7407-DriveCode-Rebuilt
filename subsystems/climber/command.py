import commands2
import constants
import constants
from subsystems.climber.subsystem import Climber
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
        self.subsystem.set_position(constants.L1_pos)

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.get_motor_position() >= constants.L1_pos
    
    def end(self, interrupted: bool):
        self.subsystem.moving = False


class Retract(commands2.Command):
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
        self.subsystem.set_voltage(constants.climber_retract_voltage)

    def execute(self):
        pass

    def isFinished(self):
        return self.subsystem.is_motor_position(constants.climber_lower_bound)

    def end(self, interrupted: bool):
        self.subsystem.moving = False
        self.subsystem.set_voltage(0)