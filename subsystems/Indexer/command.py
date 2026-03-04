import commands2

from .constants import *
from .subsystem import Indexer
from wpimath.filter import Debouncer


class RunIndexer(commands2.Command):
    """
    Runs Indexer
    """
    def __init__(self, subsystem: Indexer):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self) -> None:
        self.subsystem.run_indexer()
        self.subsystem.run_tower()
    
    def execute(self) -> None:
        pass
    
    def isFinished(self) -> None:
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop_indexer_motor()
        self.subsystem.stop_tower_motor()

class RunIndexerReversed(commands2.Command):
    """
    Runs Indexer in reverse
    """
    def __init__(self, subsystem: Indexer):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self) -> None:
        self.subsystem.run_indexer_reverse()
        self.subsystem.run_tower_reverse()
    
    def execute(self) -> None:
        pass
    
    def isFinished(self) -> None:
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop_indexer_motor()
        self.subsystem.stop_tower_motor()

class AutoUnjamming(commands2.Command):
    """
    uses a debouncer to unjam game pieces 
    """

    def __init__(self, subsystem: Indexer):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)
        self.debouncer = Debouncer(debouncer_time, Debouncer.DebounceType.kRising)

        self.counter: int = 0

    def initialize(self) -> None:
        pass
    
    def execute(self) -> None:
        if self.debouncer.calculate(
            self.subsystem.get_tower_motor_velocity() < motor_velocity_threshold 
            and self.subsystem.get_tower_motor_current() > motor_current_threshold
        ):
            self.counter = unjamming_time
        if self.counter > 0:
            self.subsystem.run_indexer_reverse()
            self.subsystem.run_tower_reverse()
        else:
            self.subsystem.run_indexer()
            self.subsystem.run_tower()
        self.counter -= 1
    
    def isFinished(self) -> None:
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop_indexer_motor()
        self.subsystem.stop_tower_motor()

class RunTower(commands2.Command):
    """
    Runs Indexer
    """
    def __init__(self, subsystem: Indexer):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self) -> None:
        self.subsystem.run_tower()
    
    def execute(self) -> None:
        pass
    
    def isFinished(self) -> None:
        return False

    def end(self, interrupted) -> None:
        self.subsystem.stop_tower_motor()