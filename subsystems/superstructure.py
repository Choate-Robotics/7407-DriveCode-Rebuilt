import commands2
from .constants import *
from subsystems import indexer, intake
from intake import SetPivot
from indexer import RunIndexer
from wpimath.filter import Debouncer

class Index(commands2.ParallelDeadlineGroup):
    def __init__(self, indexer: Indexer, intake: Intake):
        super().__init__(
            SetPivot(intake, intake_initial_angle),
            RunIndexer(indexer)
        )