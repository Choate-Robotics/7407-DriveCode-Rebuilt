import constants
from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
import math
import commands2
import wpimath.units
import ntcore

#assuming the arms are not connected, but we want them to move with eachother
class Climber(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.moving = False
        self.zeroed = False
        self.leader_motor = hardware.TalonFX(constants.leader_motor_id)
        self.follower_motor = hardware.TalonFX(constants.follower_motor_id)
        self.encoder: CANcoder = CANcoder(constants.climber_cancoder_id)
        self.motors = [self.follower_motor, self.leader_motor]

    def init(self) -> None:
        self.leader_motor_out = controls.VoltageOut(0)
        self.follower_motor_out = controls.VoltageOut(0)
        self.leader_motor.configurator.apply(constants.leader_motor_configs)
        self.follower_motor.configurator.apply(constants.follower_motor_configs)
        self.zero()

    def zero(self) -> None:
        for motor in self.motors:
            motor.set_position(0)
            self.zeroed = True

    def set_position(self, target) -> None:
        if target > 0:
            for motor in self.motors:
                motor.set_control(controls.MotionMagicVoltage(target)) 
        else:
            for motor in self.motors:
                motor.set_control(controls.VoltageOut(constants.climber_drop_voltage))
        self.moving = True
        target = 0
    
    def get_motor_revolutions(self) -> float:
        return self.leader_motor.get_position().value

    def update_table(self) -> None:
        table = ntcore.NetworkTableInstance.getDefault().getTable("climber")
        table.putBoolean("climber running", self.moving)
        table.putBoolean("climber zeroed", self.zeroed)
        table.putNumber("absolute position", self.encoder.get_absolute_position().value)
        table.putNumber("lead motor current", self.leader_motor.get_supply_current().value) 

    def periodic(self) -> None:
        self.update_table()
