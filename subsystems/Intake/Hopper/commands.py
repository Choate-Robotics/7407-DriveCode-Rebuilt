from subsystems.Intake.Hopper.Intake import Intake
import commands2
import constants
import wpilib
from enum import Enum
from utils import local_logger

log = local_logger.LocalLogger("Intake")

class SetPivotAngle(commands2.Command):
    """
    set pivot angle for intake
    """

    def __init__(self, subsystem: Intake, angle: float):
        super().__init__()
        self.subsystem = subsystem
        self.angle = angle     

    def initialize(self):
        self.subsystem.set_angle(self.angle)
        self.subsystem.wrist_moving = True

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.angle)
    
    def end(self, interrupted: bool):
        if not interrupted:
            self.subsystem.wrist_moving = False
        else:
            log.message("Intake Pivot command Interrupted")

class RunIntake(commands2.Command):
    """
    intake fuel
    """

    def __init__(self, subsystem: Intake):
        super().__init__()
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.intake_fuel()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self.subsystem.stop_intake()
        self.intake_running = False