from commands2 import ParallelCommandGroup, WaitCommand, SequentialCommandGroup
from .indexer import RunIndexer, Indexer, RunTower
from .intake import IntakeIndex, Intake, RunIntake, SetPivot, index_speed, intake_index_time, intake_retract_rotation
from .shooter import Shooter, SetShooterSlow

class Index(ParallelCommandGroup):
    def __init__(self, indexer: Indexer, intake: Intake):
        super().__init__(
            SequentialCommandGroup(
                WaitCommand(intake_index_time),
                IntakeIndex(intake),
                SetPivot(intake, intake_retract_rotation),
                RunIntake(intake, index_speed)
            ),
            RunIndexer(indexer)
        )

class ClearTower(ParallelCommandGroup):
    def __init__(self, indexer: Indexer, shooter: Shooter):
        super().__init__(
            RunTower(indexer),
            SetShooterSlow(shooter)
        )