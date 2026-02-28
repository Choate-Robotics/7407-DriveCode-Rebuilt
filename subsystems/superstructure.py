from commands2 import ParallelCommandGroup
from .indexer import RunIndexer, Indexer, RunTower
from .intake import IntakeIndex, Intake
from .shooter import Shooter, SetShooterSlow

class Index(ParallelCommandGroup):
    def __init__(self, indexer: Indexer, intake: Intake):
        super().__init__(
            IntakeIndex(intake),
            RunIndexer(indexer)
        )

class ClearTower(ParallelCommandGroup):
    def __init__(self, indexer: Indexer, shooter: Shooter):
        super().__init__(
            RunTower(indexer),
            SetShooterSlow(shooter)
        )