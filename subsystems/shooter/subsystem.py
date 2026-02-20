from .constants import *
import math
import ntcore
from utils import shooter_utils
from wpimath.geometry import Pose2d
from commands2 import Subsystem
from phoenix6 import hardware, controls, configs, signals

class Shooter(Subsystem):
    def __init__(self):
        super().__init__()
        self.left_leader_motor = hardware.TalonFX(left_lead_id)
        self.left_follower_motor = hardware.TalonFX(left_follower_id)

        self.right_leader_motor = hardware.TalonFX(right_lead_id)
        self.right_follower_motor = hardware.TalonFX(right_follow_id)

        self.velocity_torque_current = controls.VelocityTorqueCurrentFOC(0)

        self.hood_motor = hardware.TalonFX(hood_id)
        self.hood_cancoder = hardware.CANcoder(hood_cancoder_id)

        self.motion_magic = controls.MotionMagicVoltage(0)

        self.left_target_velocity = 0
        self.right_target_velocity = 0 
        self.hood_target_angle = 0

        self.hood_cancoder.configurator.apply(hood_cancoder_config)
        
        self.left_leader_motor.configurator.apply(flywheel_config.with_motor_output(
            configs.MotorOutputConfigs()
            .with_inverted(left_direction)
        ))
        self.left_follower_motor.set_control(controls.Follower(left_lead_id, signals.MotorAlignmentValue.ALIGNED))

        self.right_leader_motor.configurator.apply(flywheel_config.with_motor_output(
            configs.MotorOutputConfigs()
            .with_inverted(right_direction)
        ))
        self.right_follower_motor.set_control(controls.Follower(right_lead_id, signals.MotorAlignmentValue.ALIGNED))
        
        self.hood_motor.configurator.apply(hood_config)

        self.table = ntcore.NetworkTableInstance.getDefault().getTable("shooter")
        self.left_velocity_pub = self.table.getDoubleTopic("left velocity").publish()
        self.right_velocity_pub = self.table.getDoubleTopic("right velocity").publish()
        self.left_target_velocity_pub = self.table.getDoubleTopic("left target velocity").publish()
        self.right_target_velocity_pub = self.table.getDoubleTopic("right target velocity").publish()
        self.hood_angle_pub = self.table.getDoubleTopic("hood angle").publish()
        self.hood_target_angle_pub = self.table.getDoubleTopic("hood target angle").publish()
        self.shooter_ready_pub = self.table.getDoubleTopic("shooter ready").publish()

    def set_left_target_velocity(self, velocity: float):
        """
        sets the target velocity of the left flywheel
        
        Args:
            velocity (rotations per second): intended left flywheel velocity in rotations per second
        """
        self.left_target_velocity = velocity

        self.left_leader_motor.set_control(self.velocity_torque_current.with_velocity(self.left_target_velocity))

    def set_right_target_velocity(self, velocity: float):
        """
        sets the target velocity of the right flywheel
        
        Args:
            velocity (rotations per second): intended right flywheel velocity in rotations per second
        """
        self.right_target_velocity = velocity

        self.right_leader_motor.set_control(self.velocity_torque_current.with_velocity(self.right_target_velocity))

    def get_left_velocity(self):
        """
        obtains the current velocity of the left flywheel

        Returns:
            return_float: current left flywheel velocity in rotations per second
        """
        return self.left_leader_motor.get_velocity().value_as_double
    
    def get_right_velocity(self):
        """
        obtains the current velocity of the right flywheel

        Returns:
            return_float: current right flywheel velocity in rotations per second
        """
        return self.right_leader_motor.get_velocity().value_as_double
    
    def left_is_at_velocity(self, velocity: float):
        """
        checks if the left flywheel is at a certain velocity

        Args:
            velocity (rotations per second): velocity to be checked

        Returns:
            boolean: whether or not the left flywheel is at the velocity (true it is and false it isn't)
        """
        return abs(self.get_left_velocity() - velocity) < flywheel_threshold
    
    def right_is_at_velocity(self, velocity: float):
        """
        checks if the right flywheel is at a certain velocity

        Args:
            velocity (rotations per second): velocity to be checked

        Returns:
            boolean: whether or not the right flywheel is at the velocity (true it is and false it isn't)
        """
        return abs(self.get_right_velocity() - velocity) < flywheel_threshold
    
    def set_hood_angle(self, angle: float):
        """
        brings the hood to given angle

        Args:
            angle (radians): intended hood angle in radians
        """

        self.hood_target_angle = max(min_hood_angle, min(angle, max_hood_angle))

        rotations = self.hood_target_angle / (2 * math.pi)

        self.hood_motor.set_control(self.motion_magic.with_position(rotations))

    def get_hood_angle(self):
        """
        obtains the current position of the hood
        
        Returns:
            return_float: current hood position in radians
        """
        return self.hood_motor.get_position()
    
    def hood_is_at_angle(self, angle: float):
        """
        checks if the right hood is at a certain angle

        Args:
            angle (radians): angle to be checked

        Returns:
            boolean: whether or not the hood is at the angle (true it is and false it isn't)
        """
        return abs(self.get_hood_angle().value - angle) < hood_threshold
    
    def ready_to_shoot(self):
        """
        checks if the system is ready to shoot by checking if the left and right flywheels are at the target velocities and the hood is at the target angle
        
        Returns:
            boolean: whether or not the system is ready to shoot (true it is and false it isn't)
        """
        return self.left_is_at_velocity(self.left_target_velocity) == True and self.right_is_at_velocity(self.right_target_velocity) and self.hood_is_at_angle(self.hood_target_angle) == True
        
    def target_stationary(self, robot_pose: Pose2d, passing: bool):
        """
        gets the target hood angle and flywheel velocity shooting when stationary
        
        :param robot_pose: robot's pose on the field
        """
        if passing:
            hood_deg, rps = shooter_utils.pass_setpoints_from_pose(robot_pose)
        else:
            hood_deg, rps = shooter_utils.shot_setpoints_from_pose(robot_pose)

        self.set_hood_angle(math.radians(hood_deg))
        self.set_left_target_velocity(rps)
        self.set_right_target_velocity(rps)
        
    def update_table(self):
        table = ntcore.NetworkTableInstance.getDefault().getTable("shooter")

        self.left_velocity_pub.set(self.get_left_velocity())
        self.right_velocity_pub.set(self.get_right_velocity())
        self.left_target_velocity_pub.set(self.left_target_velocity)
        self.right_target_velocity_pub.set(self.right_target_velocity)
        self.hood_angle_pub.set(self.get_hood_angle())
        self.hood_target_angle_pub.set(self.hood_target_angle)
        self.shooter_ready_pub.set(self.ready_to_shoot())

    def periodic(self):
        if NT_SHOOTER:
            self.update_table