from .constants import *
from phoenix6 import controls, hardware
import commands2

class Climber(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = hardware.TalonFX(left_motor_id)
        self.motor.configurator.apply(climber_motor_configs)
        self.motor_out = controls.VoltageOut(0)

        self.zero()
        self.zeroed = True

    def zero(self):
        self.motor.set_position(0)

    def set_voltage(self, voltage):
        self.motor.set_control(self.motor_out.with_output(voltage))
        self.moving = True

    def get_motor_position(self):
        return self.motor.get_rotor_position().value
    
    def is_motor_position(self, position) -> bool:
        return (self.get_motor_position() >= position)


    