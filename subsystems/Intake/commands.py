from Intake import Intake
import commands
import commands2
from enum import Enum
from utils import local_logger

log = local_logger.LocalLogger("intake")

class deploy_intake(commands2.Command):
    def __init__(self, subsystem: Intake, angle):
        super().__init__()
        self.subsystem = subsystem
        self.angle = angle
        self.addRequirements(self.subsystem)

    def initialize(self):
        self.subsystem.set_pivot(self.angle)
        self.subsystem.pivot_running = True

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.angle)
    
    def end(self, interrupted: bool):
        if not interrupted:
            self.subsystem.pivot_running = False
        else:
            log.message("intake pivot interrupted")

class run_intake(commands2.Command):
    def __init__(self, subsystem: Intake):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self):
        self.subsystem.intake_fuel()

    def isFinished(self) -> bool:
        "command expected to be interrupted"
        return False
    
    def end(self, interrupted: bool):
        self.subsystem.stop_intake() 

class reverse_intake(commands2.Command):
    def __init__(self, subsystem: Intake):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self):
        self.subsystem.reverse_intake()

    def isFinished(self) -> bool:
        "command expected to be interrupted"
        return False
    
    def end(self, interrupted: bool):
        self.subsystem.stop_intake() 