from subsystems.Intake.Hopper.Intake import Intake, Hopper
import commands2
import constants
import wpilib
from enum import Enum
from utils import local_logger

class Deployment(commands2.SequentialCommandGroup):
    """
    i dont know how to go about deployment
    its not going to be a sequential command group because the pivot and the hopper are "tied" together
    it's probably going to be a wrapper command of setpivotangle icl
    this file is just here so u can read my thoughts :)
    """
    def __init__(self, subsystem: Intake, Hopper):
        subsyste