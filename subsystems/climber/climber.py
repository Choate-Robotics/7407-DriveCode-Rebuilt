import constants
from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
import math
import commands2
import wpimath.units


class climber(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.moving = False
        self.zeroed = False
        self.leader_motor = hardware.TalonFX(constants.leader_motor_id)
        self.follower_motor = hardware.TalonFX(constants.follower_motor_id)
        self.leader_motor_out = controls.DutyCycleOut(0) #do I need dutycycle for these motors??
        self.follower_motor.set_control(controls.follower.Follower(constants.leader_motor_id, signals.MotorAlignmentValue(1))) #check with Eben

    def init(self):
        self.leader_motor.configurator.apply(constants.leader_motor_configs)
    

    def zero(self):
        self.leader_motor.set_position(0)
        self.zeroed = True

    def set_raw_output(self, raw_value: float):
        self.set_raw_output(raw_value)
    
    def get_motor_revolutions(self):
        return self.leader_motor.get_position()

