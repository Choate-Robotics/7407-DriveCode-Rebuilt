from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
import math
import commands2
import constants
# from units.SI import radians


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.encoder: CANcoder = CANcoder(constants.intake_cancoder_id, '5')
        self.horizontal_motor = hardware.TalonFX(constants.horizontal_motor_id)

        self.pivot_motor = hardware.TalonFX(constants.pivot_motor_id)
        self.horizontal_motor_out = controls.DutyCycleOut(0)
        self.pivot_request = controls.DutyCycleOut(0.0)
    
        self.pivot_motor_zeroed = False

    def init(self):
        self.horizontal_motor.configurator.apply(constants.horizontal_motor_configs)
        self.pivot_motor.configurator.apply(constants.pivot_motor_configs)

    def intake_fuel(self):
        self.horizontal_motor.set_control(self.horizontal_motor_out.with_output(constants.intake_fuel_speed))

    def get_pivot_motor_current(self):
        return self.pivot_motor.get_supply_current()
    
    def get_horizontal_motor_current(self):
        return self.horizontal_motor.get_supply_current()
    
    def zero_pivot(self):
        self.pivot_angle = (
            (self.encoder.get_absolute_position().value))
        pos = self.pivot_angle
        self.pivot_motor.set_position(pos)
        self.pivot_motor_zeroed = True

    def get_pivot_angle(self):
        return (self.pivot_motor.get_position().value)
       
    def stop_pivot(self):
        self.pivot_motor.set_control(self.pivot_request.with_output(0.0))
        
    def is_at_angle(self, angle: float):
        return abs(self.get_pivot_angle() - angle) < constants.intake_angle_threshold

    def deploy_intake(self, angle:float):
        self.pivot_motor.set_position(angle)


