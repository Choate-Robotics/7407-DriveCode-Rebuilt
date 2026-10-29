import commands2
from .constants import *
from .subsystem import Climber

class DeployClimb(commands2.Command):
    def __init__(self, subsystem:Climber):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)


    def initialize(self):
        pass

    def execute(self):
        self.subsystem.set_voltage(climber_extend_voltage)

    def isFinished(self):
        return (self.subsystem.get_motor_position() >= climber_upper_bound)

    def end(self, interrupted: bool):
        self.subsystem.set_voltage(0)
        self.subsystem.moving = False
