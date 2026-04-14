import commands2
from .subsystem import Shooter
from wpimath.geometry import Pose2d
from subsystems.drivetrain.command_swerve_drivetrain import CommandSwerveDrivetrain
from utils import alliance_flip_util, field_constants, shooter_utils
from .constants import *
from phoenix6 import units

from ntcore import NetworkTableInstance

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
        pose = self.drivetrain.get_pose()
        speeds = self.drivetrain.get_speeds()

        passing = alliance_flip_util.get_x(pose.X()) >= field_constants.LinesVertical.NEUTRAL_ZONE_NEAR

        if passing and abs(speeds.vx) > SOM_VELOCITY_THRESHOLD:
            shifted_target = shooter_utils.get_pass_on_move_target(pose, speeds)
            hood_deg, rps = shooter_utils.pass_setpoints_from_target(pose, shifted_target)

            self.subsystem.set_hood_angle(hood_deg / 360)
            self.subsystem.set_target_velocity(rps)
        else:
            self.subsystem.target_stationary(pose, passing)

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

    def __init__(self, subsystem: Shooter, drivetrain: CommandSwerveDrivetrain):
        super().__init__()

        self.drivetrain = drivetrain
        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self):
        pass

    def execute(self):
        self.subsystem.target_stationary(self.drivetrain.get_pose(), False)

    def isFinished(self) -> bool:
        """
        End command for autonomous purposes
        """
        return False

    def end(self, interrupted):
        pass

class SetShooterIdle(commands2.Command):
    def __init__(self, subsystem: Shooter):
        super().__init__()   

        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self):
        self.subsystem.set_target_velocity(idle_velocity)
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
            angle (rotations): desired hood angle
    """

    def __init__(self, subsystem: Shooter, velocity: units.rotations_per_second, angle: units.rotation):
        super().__init__()

        self.subsystem = subsystem
        self.addRequirements(self.subsystem)
        self.velocity = velocity
        self.angle = angle

    def initialize(self):
        self.subsystem.set_target_velocity(self.velocity)
        self.subsystem.set_hood_angle(self.angle)

    def execute(self):
        pass

    def isFinished(self):
        return False

    def end(self, interrupted):
        pass

class SetShooterSlow(commands2.Command):
    def __init__(self, subsystem: Shooter):
        super().__init__()

        self.subsystem = subsystem
        self.addRequirements(self.subsystem)

    def initialize(self):
        self.subsystem.set_target_velocity(slow_velocity)
        self.subsystem.set_hood_angle(hood_clear_angle)

    def execute(self):
        pass

    def isFinished(self):
        return False

    def end(self, interrupted):
        pass


class TuneShooter(commands2.Command):
    def __init__(self, subsystem: Shooter, drivetrain: CommandSwerveDrivetrain):
        super().__init__()

        self.subsystem = subsystem
        self.addRequirements(self.subsystem)
        self.drivetrain = drivetrain

        self.nt_inst = NetworkTableInstance.getDefault()
        self.shot_tuner = self.nt_inst.getTable("Shot Tuner")

        
        self.hood_angle_pub = self.shot_tuner.getDoubleTopic("hood angle").publish()
        self.flywheel_rps_pub = self.shot_tuner.getDoubleTopic("flywheel rps").publish()
        self.hood_angle_sub = self.shot_tuner.getDoubleTopic("hood angle").subscribe(min_hood_angle)
        self.flywheel_rps_sub = self.shot_tuner.getDoubleTopic("flywheel rps").subscribe(0)
        

    def initialize(self):
        pass

    def execute(self):
        self.subsystem.set_target_velocity(self.flywheel_rps_sub.get())
        self.subsystem.set_hood_angle(self.hood_angle_sub.get() / 360)

    def isFinished(self):
        return False

    def end(self, interrupted):
        pass