from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
import math
import commands2
import constants
# from units.SI import radians
import ntcore


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        # self.encoder: CANcoder = CANcoder()
        self.horizontal_motor = hardware.TalonFX(constants.horizontal_motor_id)

        self.pivot_motor = hardware.TalonFX(constants.pivot_motor_id)
        self.horizontal_motor_out = controls.DutyCycleOut(0)
        self.pivot_request = controls.MotionMagicDutyCycle(0.0)
        self.target_angle = 0.0
    
        self.pivot_motor_zeroed = False
        self.intake_running = False
        self.pivot_running = False

    def init(self):
        self.horizontal_motor.configurator.apply(constants.horizontal_motor_configs)
        self.pivot_motor.configurator.apply(constants.pivot_motor_configs)
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("Intake")
        self.anglepub = self.table.getDoubleTopic("pivot angle").publish()
        self.zeroedpub = self.table.getBooleanTopic("pivot zeroed").publish()
        self.targetpub = self.table.getDoubleTopic("target angle").publish()
        self.pivot_currentpub = self.table.getDoubleTopic("pivot current").publish()
        self.horizontal_motor_currentpub = self.table.getDoubleTopic("horizontal motor current").publish()
        self.intake_runningpub = self.table.getBooleanTopic("intake running").publish()
        

    def intake_fuel(self):
        self.horizontal_motor.set_control(self.horizontal_motor_out.with_output(constants.fuel_speed))
        self.intake_running = True

    def reverse_intake(self):
        self.horizontal_motor.set_control(self.horizontal_motor_out.with_output(-constants.fuel_speed))
        self.intake_running = True

    def stop_intake(self):
        self.horizontal_motor.set_control(self.horizontal_motor_out.with_output(0.0))

    def get_pivot_motor_current(self):
        return self.pivot_motor.get_supply_current()
    
    def get_horizontal_motor_current(self):
        return self.horizontal_motor.get_supply_current()
    
    """
    def zero_pivot(self):
        self.pivot_angle = (
            (self.encoder.get_absolute_position().value))
        pos = self.pivot_angle
        self.pivot_motor.set_position(pos)
        self.pivot_motor_zeroed = True
    """ # not sure what to do with this -> https://github.com/Choate-Robotics/7407-DriveCode-Rebuilt/pull/2#discussion_r2722981005

    def get_pivot_angle(self):
        return (self.pivot_motor.get_position().value)
       
    def stop_pivot(self):
        self.pivot_request = controls.MotionMagicDutyCycle(0.0)
        self.pivot_motor.set_control(self.pivot_request)
        
    def is_at_angle(self, angle: float):
        return abs(self.get_pivot_angle() - angle) < constants.angle_threshold

    def set_pivot(self, angle: float):
        self.target_angle = angle
        self.pivot_request = self.pivot_request = controls.MotionMagicDutyCycle(self.target_angle)
        self.pivot_motor.set_control(self.pivot_request)
    
    def update_table(self):
        self.anglepub.set(self.get_pivot_angle())
        self.targetpub.set(self.target_angle)
        self.pivot_currentpub.set(self.get_pivot_motor_current().value)
        self.intake_runningpub.set(self.intake_running)
        self.horizontal_motor_currentpub.set(self.get_horizontal_motor_current().value)