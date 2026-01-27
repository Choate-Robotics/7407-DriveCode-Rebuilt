import commands2
from subsystem import Shooter
from phoenix6.units import rotations_per_second, radian

class SetShooter(commands2.Command):
    """
    set left and right flywheels to specified velocity and set hood to specified angle.
    never ends
    
    Args:
            velocity (rotations per second): intended left flywheel velocity in rotations per second
            angle (radians): intended hood angle in radians
    """

    def __init__(self, subsystem: Shooter, velocity: rotations_per_second, angle: radian):
        super().__init__()
        self.addRequirements(Shooter)

        self.subsystem = subsystem
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

    def __init__(self, subsystem: Shooter, velocity, angle):
        super().__init__()
        self.addRequirements(Shooter)

        self.subsystem = subsystem
        self.velocity = velocity
        self.angle = angle


    def initialize(self):
        self.subsystem.set_left_target_velocity(self.velocity)
        self.subsystem.set_right_target_velocity(self.velocity)
        self.subsystem.set_hood_angle(self.angle)

    def execute(self):
        pass

    def isFinished(self) -> bool:
        """
        we expect this command to be interrupted
        """
        return self.subsystem.ready_to_shoot()

    def end(self):
        pass