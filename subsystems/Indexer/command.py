from commands2.command import SubsystemCommand

import constants
from subsystem import Indexer
from utils import LocalLogger
from wpilib.filter import Debouncer


log = LocalLogger("Indexer command")

class RunIndexer(SubsystemCommand[Indexer]):
    """
    Runs Indexer
    """
    def __init__(self, subsystem: Indexer):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.run_indexer()
        self.subsystem.run_tower()
    
    def execute(self) -> None:
        pass
    
    def isFinished(self) -> None:
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop()

class RunIndexerReversed(SubsystemCommand[Indexer]):
    """
    Runs Indexer in reverse
    """
    def __init__(self, subsystem: Indexer):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.run_indexer_reverse()
        self.subsystem.run_tower_reverse()
    
    def execute(self) -> None:
        pass
    
    def isFinished(self) -> None:
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop()

class AutoUnjamming(SubsystemCommand[Indexer]):
    """
    uses a debouncer to unjam game pieces 
    """

    def __init__(self, subsystem: Indexer):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.debouncer = Debouncer(constants.debouncer_time, Debouncer.DebounceType.kRising)
        self.tower_velocity = self.get_tower_motor_velocity() < constants.motor_velocity_threshold
        self.tower_current = self.get_tower_motor_current() > constants.motor_current_threshold

    def initialize(self) -> None:
        if self.debouncer.calculate(self.tower_velocity and self.tower_current):
            self.subsystem.run_indexer_reverse()
            self.subsystem.run_tower_reverse()
    
    def execute(self) -> None:
        pass
    
    def isFinished(self) -> None:
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop()