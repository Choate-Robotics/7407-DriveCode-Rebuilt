from .subsystem import Intake
import commands2
from .constants import *
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
            self.subsystem.stop_pivot()
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

class SetPivotIn(commands2.Command):
    """
    Set pivot motor to specified angle with voltage in
    """
    def __init__(self, subsystem: Intake, angle: float):
        super().__init__()
        self.subsystem = subsystem
        self.angle = angle

    def initialize(self):
        self.subsystem.set_pivot_motor_in(voltage_out)
        self.subsystem.pivot_running = True

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.angle)
    
    def end(self, interrupted: bool):
        if not interrupted:
            self.subsystem.stop_pivot()
            self.subsystem.pivot_running = False
        else:
            log.message("intake pivot interrupted")

class DeployIntake(SetPivot):
    """
    Deploy the intake
    """
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem, intake_deploy_angle)


class DeployIntakeOut(commands2.SequentialCommandGroup):
    """
    Deploy intake by setting pivot to specified angle with voltageout
    """
    def __init__(self, subsystem: Intake, angle: float):
        super().__init__()
        self.command = SetPivotIn(subsystem, intake_deploy_angle)

    def initialize(self):
        self.command.initialize()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.command.isFinished()
    
    def end(self, interrupted: bool):
        self.command.end(interrupted)