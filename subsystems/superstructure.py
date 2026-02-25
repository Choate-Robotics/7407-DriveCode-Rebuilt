from commands2 import ParallelCommandGroup
from .indexer import RunIndexer, Indexer
from .intake import IntakeIndex, Intake

class Index(ParallelCommandGroup):
    def __init__(self, indexer: Indexer, intake: Intake):
        super().__init__(
            IntakeIndex(intake),
            RunIndexer(indexer)
        )