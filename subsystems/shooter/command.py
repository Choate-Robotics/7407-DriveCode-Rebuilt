import commands2
from shooter import Shooter
from wpimath.geometry import Pose2d
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

class SetShooter(commands2.Command):
    """
    set left and right flywheels to specified velocity and set hood to specified angle.
    never ends
    
    Args:
            pose: robot Pose2d
    """

    def __init__(self, subsystem: Shooter, drivetrain: CommandSwerveDrivetrain, pose: Pose2d):
        super().__init__()
        self.addRequirements(Shooter)

        self.subsystem = subsystem
        self.pose = pose

    def initialize(self):
        self.subsystem.target_stationary(CommandSwerveDrivetrain.get_pose())

    def execute(self):
        pass

    def isFinished(self):
        return False

    def end(self):
        pass

class SetShooterAuto(commands2.Command):
    """
    set left and right flywheels to specified velocity and set hood to specified angle.
    ends
    
    Args:
            velocity (rotations per second): intended left flywheel velocity in rotations per second
            angle (radians): intended hood angle in radians
    """

    def __init__(self, subsystem: Shooter, pose: Pose2d):
        super().__init__()
        self.addRequirements(Shooter)

        self.subsystem = subsystem
        self.pose = pose

    def initialize(self):
        self.subsystem.target_stationary(Pose2d)

    def execute(self):
        pass

    def isFinished(self) -> bool:
        """
        we expect this command to be interrupted
        """
        return self.subsystem.ready_to_shoot()

    def end(self):
        pass