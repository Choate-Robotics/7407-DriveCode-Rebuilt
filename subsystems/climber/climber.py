import constants
from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
import math
import commands2
import wpimath.units

#assuming the arms are connected
class Climber(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.moving = False
        self.zeroed = False
        self.leader_motor = hardware.TalonFX(constants.leader_motor_id)
        self.follower_motor = hardware.TalonFX(constants.follower_motor_id)
        
        self.motors = [self.follower_motor, self.leader_motor]

    def init(self):
        self.leader_motor_out = controls.VoltageOut(0)
        self.follower_motor_out = controls.VoltageOut(0)
        self.leader_motor.configurator.apply(constants.leader_motor_configs)
        self.follower_motor.configurator.apply(constants.follower_motor_configs)
        self.zero()

    def zero(self):
        for motor in self.motors:
            motor.set_position(0)
            self.zeroed = True

    def set_target_position(self, pos, rotations):

        for motor in self.motors:
            motor.set_control(controls.MotionMagicVoltage.with_position(pos,rotations)) #idk how to do PID
        self.moving = True


    
    def get_motor_revolutions(self):
        return self.leader_motor.get_position()

