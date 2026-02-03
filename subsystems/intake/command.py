from subsystem import Intake
import subsystems.intake.command as command
import commands2
import constants
from enum import Enum
from utils import local_logger

log = local_logger.LocalLogger("intake")

class SetPivot(commands2.Command):
    """
    Setpivot to specificed angle 
    """
    def __init__(self, subsystem: Intake, angle: float):
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

class RunIntake(commands2.Command):
    """
    Run intake
    """
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

class ReverseIntake(commands2.Command):
    """
    Reverse intake
    """
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


class DeployIntake(commands2.SequentialCommandGroup):
    """
    Deploy intake by setting pivot to specificed angle and running intake
    """
    def __init__(self, subsystem: Intake):
        super().__init__(
            SetPivot(subsystem, constants.deploy_angle),
            RunIntake(subsystem)
        )
        self.subystem = subsystem
        self.addRequirements(subsystem)