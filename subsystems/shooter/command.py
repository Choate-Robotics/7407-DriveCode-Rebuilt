import commands2
from shooter import Shooter
from wpimath.geometry import Pose2d
from subsystems.drivetrain.command import CommandSwerveDrivetrain
from utils import alliance_flip_util, field_constants
import constants

class AimShooter(commands2.Command):
    """
    uses target_stationary function to set left and right flywheels to specified velocity and set hood to specified angle.
    never ends
    
    Args:
            pose: robot Pose2d
    """

    def __init__(self, subsystem: Shooter, drivetrain: CommandSwerveDrivetrain):
        super().__init__()     

        self.addRequirements(self.subsystem)

        self.subsystem = subsystem
        self.drivetrain = drivetrain

    def initialize(self):
        pass

    def execute(self):
        """
        if you are in passing zone, set shooter to passing setpoints
        else, set shooter to shooting setpoints
        """
        if alliance_flip_util.get_x(self.drivetrain.get_pose().X()) < field_constants.LinesVertical.ALLIANCE_ZONE:
            self.subsystem.target_stationary(self.drivetrain.get_pose(), False)
        else:
            self.subsystem.target_stationary(self.drivetrain.get_pose(), True)

    def isFinished(self):
        return False

    def end(self):
        pass
class SetShooterAuto(commands2.Command):
    """
    uses target_stationary function to set left and right flywheels to specified velocity and set hood to specified angle.
    ends
    
    Args:
            pose: robot Pose2d
    """

    def __init__(self, subsystem: Shooter, pose: Pose2d):
        super().__init__()
       
       
        self.addRequirements(self.subsystem)

        self.subsystem = subsystem
        self.pose = pose

    def initialize(self):
        self.subsystem.target_stationary(Pose2d, False)

    def execute(self):
        pass

    def isFinished(self) -> bool:
        """
        we expect this command to be interrupted
        """
        return self.subsystem.ready_to_shoot()

    def end(self):
        pass

class Idle(commands2.Command):
    def __init__(self, subsystem: Shooter):
        super().__init__()   

        self.addRequirements(self.subsystem)

        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.set_left_target_velocity(constants.idle_velocity)
        self.subsystem.set_right_target_velocity(constants.idle_velocity)
        self.subsystem.set_hood_angle(constants.min_hood_angle)

    def execute(self):
        pass

    def isFinished(self):
        return False

    def end(self):
        pass