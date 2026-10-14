from robot_constants import NT_UPDATE_EVERY
from .constants import *
import math
import ntcore
from utils import shooter_utils
from utils.phoenix_util import apply_config
from wpimath.geometry import Pose2d
from commands2 import Subsystem
from phoenix6 import hardware, controls, configs, signals

class Shooter(Subsystem):
    def __init__(self):
        super().__init__()
        self._nt_loop = 0
        self.left_leader_motor = hardware.TalonFX(left_lead_id)
        self.left_follower_motor = hardware.TalonFX(left_follower_id)

        self.right_follower_motor_1 = hardware.TalonFX(right_lead_id)
        self.right_follower_motor_2 = hardware.TalonFX(right_follow_id)

        self.velocity_torque_current = controls.VelocityTorqueCurrentFOC(0)
        self.duty_cycle = controls.DutyCycleOut(0)

        self.hood_motor = hardware.TalonFX(hood_id)
        self.hood_cancoder = hardware.CANcoder(hood_cancoder_id)

        self.motion_magic = controls.PositionVoltage(0)

        self.left_target_velocity = 0
        self.hood_target_angle = 0

        self.flywheel_trim = 0
        self.hood_trim = 0  # degrees

        apply_config(self.hood_cancoder, hood_cancoder_config)
        apply_config(self.hood_motor, hood_config)
        
        apply_config(self.left_leader_motor, left_flywheel_config)
        self.left_follower_motor.set_control(controls.Follower(left_lead_id, signals.MotorAlignmentValue.ALIGNED))

        self.right_follower_motor_1.set_control(controls.Follower(left_lead_id, signals.MotorAlignmentValue.OPPOSED))
        self.right_follower_motor_2.set_control(controls.Follower(left_lead_id, signals.MotorAlignmentValue.OPPOSED))
        

        self.table = ntcore.NetworkTableInstance.getDefault().getTable("shooter")
        self.left_velocity_pub = self.table.getDoubleTopic("left velocity").publish()
        self.left_target_velocity_pub = self.table.getDoubleTopic("left target velocity").publish()
        self.hood_angle_pub = self.table.getDoubleTopic("hood angle").publish()
        self.hood_target_angle_pub = self.table.getDoubleTopic("hood target angle").publish()
        self.shooter_ready_pub = self.table.getBooleanTopic("shooter ready").publish()
        # actual - target, published by the robot since Elastic can't combine topics
        self.left_velocity_error_pub = self.table.getDoubleTopic("left velocity error").publish()
        self.hood_angle_error_pub = self.table.getDoubleTopic("hood angle error").publish()
        # Operator's flywheel trim (rps), published when it changes
        self.flywheel_trim_pub = self.table.getDoubleTopic("flywheel trim").publish()
        self.flywheel_trim_pub.set(self.flywheel_trim)
        # Operator's hood trim (degrees), published when it changes
        self.hood_trim_pub = self.table.getDoubleTopic("hood trim").publish()
        self.hood_trim_pub.set(self.hood_trim)

    def set_target_velocity(self, velocity: units.rotations_per_second):
        """
        sets the target velocity of the left flywheel
        
        Args:
            velocity (rotations per second): intended left flywheel velocity in rotations per second
        """
        self.left_target_velocity = velocity

        self.left_leader_motor.set_control(self.velocity_torque_current.with_velocity(self.left_target_velocity))

    def stop(self):
        self.left_target_velocity = 0
        self.left_leader_motor.set_control(self.duty_cycle.with_output(0))
        
    def get_velocity(self) -> units.rotations_per_second:
        """
        obtains the current velocity of the left flywheel

        Returns:
            return_float: current left flywheel velocity in rotations per second
        """
        return self.left_leader_motor.get_velocity().value_as_double
    
    def is_at_velocity(self, velocity: units.rotations_per_second):
        """
        checks if the left flywheel is at a certain velocity

        Args:
            velocity (rotations per second): velocity to be checked

        Returns:
            boolean: whether or not the left flywheel is at the velocity (true it is and false it isn't)
        """
        return abs(self.get_velocity() - velocity) < flywheel_velocity_threshold
    
    def set_hood_angle(self, angle: units.rotation):
        """
        brings the hood to given angle

        Args:
            angle (rotations): intended hood angle in rotations
        """

        self.hood_target_angle = max(min_hood_angle, min(angle, max_hood_angle))
        self.hood_motor.set_control(self.motion_magic.with_position(self.hood_target_angle))

    def get_hood_angle(self):
        """
        obtains the current position of the hood
        
        Returns:
            return_float: current hood position in rotations
        """
        return self.hood_motor.get_position()
    
    def hood_is_at_angle(self, angle: units.rotation):
        """
        checks if the right hood is at a certain angle

        Args:
            angle (rotations): angle to be checked

        Returns:
            boolean: whether or not the hood is at the angle (true it is and false it isn't)
        """
        return abs(self.get_hood_angle().value - angle) < hood_angle_threshold
    
    def ready_to_shoot(self):
        """
        checks if the system is ready to shoot by checking if the left and right flywheels are at the target velocities and the hood is at the target angle
        
        Returns:
            boolean: whether or not the system is ready to shoot (true it is and false it isn't)
        """
        return self.is_at_velocity(self.left_target_velocity) and self.hood_is_at_angle(self.hood_target_angle)
        
    def target_stationary(self, robot_pose: Pose2d, passing: bool):
        """
        gets the target hood angle and flywheel velocity shooting when stationary
        
        :param robot_pose: robot's pose on the field
        """
        if passing:
            hood_deg, rps = shooter_utils.pass_setpoints_from_pose(robot_pose)
        else:
            hood_deg, rps = shooter_utils.shot_setpoints_from_pose(robot_pose)

        self.set_hood_angle((hood_deg + self.hood_trim)/360)  # clamped to the hood limits there
        self.set_target_velocity(rps + self.flywheel_trim)

    def trim_down(self):
        self.flywheel_trim -= flywheel_trim_amount
        self.flywheel_trim_pub.set(self.flywheel_trim)

    def trim_up(self):
        self.flywheel_trim += flywheel_trim_amount
        self.flywheel_trim_pub.set(self.flywheel_trim)

    def trim_hood_down(self):
        self.hood_trim -= hood_trim_amount
        self.hood_trim_pub.set(self.hood_trim)

    def trim_hood_up(self):
        self.hood_trim += hood_trim_amount
        self.hood_trim_pub.set(self.hood_trim)
        
    def update_table(self):
        velocity = self.get_velocity()
        hood_angle = self.get_hood_angle().value * 360
        self.left_velocity_pub.set(velocity)
        self.left_target_velocity_pub.set(self.left_target_velocity)
        self.hood_angle_pub.set(hood_angle)
        self.hood_target_angle_pub.set(self.hood_target_angle * 360)
        # Errors only while shooting (a real target, not idle or the slow spin), so the
        # dashboard graphs show shots instead of the gap between idle and the last target.
        if self.left_target_velocity > slow_velocity:
            self.left_velocity_error_pub.set(velocity - self.left_target_velocity)
            self.hood_angle_error_pub.set(hood_angle - self.hood_target_angle * 360)
        self.shooter_ready_pub.set(self.ready_to_shoot())

    def periodic(self):
        self._nt_loop = (self._nt_loop + 1) % NT_UPDATE_EVERY
        if NT_SHOOTER and self._nt_loop == 0:
            self.update_table()