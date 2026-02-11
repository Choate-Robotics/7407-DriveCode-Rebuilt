import commands2
from .subsystem import Shooter
from wpimath.geometry import Pose2d
from subsystems.drivetrain.command_swerve_drivetrain import CommandSwerveDrivetrain
from utils import alliance_flip_util, field_constants
from .constants import *

class AimShooter(commands2.Command):
    """
    uses target_stationary function to set left and right flywheels to specified velocity and set hood to specified angle.
    never ends
    
    Args:
            pose: robot Pose2d
    """

    def __init__(self, subsystem: Shooter, drivetrain: CommandSwerveDrivetrain):
        super().__init__()     

        self.subsystem = subsystem
        self.addRequirements(self.subsystem)
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

    def end(self, interrupted):
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
       
       
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)
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

    def end(self, interrupted):
        pass

class SetShooterIdle(commands2.Command):
    def __init__(self, subsystem: Shooter):
        super().__init__()   

        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self):
        self.subsystem.set_left_target_velocity(idle_velocity)
        self.subsystem.set_right_target_velocity(idle_velocity)
        self.subsystem.set_hood_angle(min_hood_angle)

    def execute(self):
        pass

    def isFinished(self):
        return False

    def end(self, interrupted):
        pass

class SetShooter(commands2.Command):
    """
    sets left and right flywheels to specified velocity and set hood to specified angle
    never ends
    
    Args:
            velocity (rotations per second): desired left and right flywheel velocity
            angle (radians): desired hood angle
    """

    def __init__(self, subsytem: Shooter, velocity: float, angle: float):
        super().__init__()

        self.subsystem = subsytem
        self.addRequirements(self.subsystem)
        self.velocity = velocity
        self.angle = angle

    def initialize(self):
        self.subsystem.set_left_target_velocity(self.velocity)
        self.subsystem.set_right_target_velocity(self.velocity)
        self.subsystem.set_hood_angle(self.angle)

    def execute(self):
        pass

    def isFinished(self):
        return False

    def end(self, interrupted):
        pass