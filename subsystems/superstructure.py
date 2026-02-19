import commands2
from subsystems.intake import constants
from subsystems import indexer, intake
from subsystems.intake import SetPivotIn
from subsystems.indexer import RunIndexer

class Index(commands2.ParallelDeadlineGroup):
    def __init__(self, indexer: indexer.Indexer, intake: intake.Intake):
        super().__init__(
            SetPivotIn(intake, constants.intake_deploy_angle),
            RunIndexer(indexer)
        )