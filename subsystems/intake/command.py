from .subsystem import Intake
import commands2
from .constants import *
from enum import Enum
from utils import local_logger
from phoenix6 import units

log = local_logger.LocalLogger("intake")

class SetPivot(commands2.Command):
    """
    Setpivot to specificed angle 
    """
    def __init__(self, subsystem: Intake, angle: units.rotation):
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
    def __init__(self, subsystem: Intake, speed=fuel_speed):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)
        self.speed = speed

    def initialize(self):
        self.subsystem.intake_fuel(self.speed)

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
    def __init__(self, subsystem: Intake, angle: units.rotation, speed=index_speed):
        super().__init__()
        self.subsystem = subsystem
        self.angle = angle
        self.speed = speed

    def initialize(self):
        self.subsystem.intake_fuel(self.speed)
        self.subsystem.set_pivot_motor_in(voltage_out)
        self.subsystem.pivot_running = True

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.subsystem.get_pivot_angle() >= self.angle
    
    def end(self, interrupted: bool):
        self.subsystem.stop_intake()
        self.subsystem.set_pivot(self.angle)
        self.subsystem.pivot_running = False
        if interrupted:
            log.message("intake pivot interrupted")

class IntakeIndex(SetPivot):
    """
    Fully retract the intake
    """
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem, intake_retract_rotation)

class RetractIntake(SetPivot):
    """
    Fully retract the intake
    """
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem, intake_maximum_rotation)

class DeployIntake(SetPivot):
    """
    Fully deploy intake
    """
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem, intake_deploy_rotation)

# class IntakeIndex(SetPivotIn):
#     """
#     Index with the intake by setting pivot to specified angle with voltagein
#     """
#     def __init__(self, subsystem: Intake):
#         super().__init__(subsystem, intake_retract_rotation)

