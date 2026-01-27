from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
import math
import commands2
import constants
# from units.SI import radians
import ntcore
from utils import local_logger

class Intake(commands2.Subsystem):
    def __init__(self):
        """
        initialize intake class
        """
        super().__init__()
        # self.encoder: CANcoder = CANcoder()
        self.horizontal_motor = hardware.TalonFX(constants.horizontal_motor_id)

        self.pivot_motor = hardware.TalonFX(constants.pivot_motor_id)
        self.horizontal_motor_out = controls.DutyCycleOut(0)
        self.pivot_request = controls.MotionMagicVoltage(0)
        self.target_angle = 0.0
    
        self.pivot_motor_zeroed = False
        self.intake_running = False
        self.wrist_moving = False
    

    def init(self):
        """
        start motors and set publishers for network tables
        """
        self.horizontal_motor.configurator.apply(constants.horizontal_motor_configs)
        self.pivot_motor.configurator.apply(constants.pivot_motor_configs)
        self.table = ntcore.NetworkTableInstance.getdefault().getTable("Intake")
        self.anglepub = self.table.getDoubleTopic("pivot angle").publish()
        self.zeroedpub = self.table.getBooleanTopic("pivot zeroed").publish()
        self.targetpub = self.table.getDoubleTopic("target angle").publish()
        self.pivot_currentpub = self.table.getDoubleTopic("pivot current").publish()
        self.horizontalmotor_currentpub = self.table.getDoubleTopic("horizontal motor current").publish()
        self.intake_runningpub = self.table.getBooleanTopic("intake running").publish()
        

    def intake_fuel(self):
        """
        set rawoutput of horizontal motor to intake fuel
        """
        self.horizontal_motor.set_control(self.horizontal_motor_out.with_output(constants.intake_fuel_speed))
        self.intake_running = True

    def get_pivot_motor_current(self):
        """
        get supply current of pivot motor
        """
        return self.pivot_motor.get_supply_current()
    
    def get_horizontal_motor_current(self):
        """
        get supply current of horizontal motor
        """
        return self.horizontal_motor.get_supply_current()
    
    """
    def zero_pivot(self):
        self.pivot_angle = (
            (self.encoder.get_absolute_position().value))
        pos = self.pivot_angle
        self.pivot_motor.set_position(pos)
        self.pivot_motor_zeroed = True
    """    

    def get_pivot_angle(self):
        """
        returns current pivot angle
        """
        return (self.pivot_motor.get_position().value)
       
    def stop_intake(self):
        """
        stops horizontal motor
        """
        self.horizontal_motor.set_control(self.horizontal_motor_out.with_output(0.0))

    def stop_pivot(self):
        """
        stops pivot motor at current position
        """
        self.pivot_motor.set_control(self.pivot_request.with_position(self.get_pivot_angle()))
        
    def is_at_angle(self, angle: float):
        """
        returns bool. checks if pivot motor is at angle
        """
        return abs(self.get_pivot_angle() - angle) < constants.intake_angle_threshold

    def set_angle(self, angle:float):
        """
        sets pivot angle
        """
        self.target_angle = angle
        self.pivot_motor.set_control(self.pivot_request.with_position(self.target_angle))
    
    def update_table(self):
        """
        periodically updates publishers
        """
        self.anglepub.set(self.get_pivot_angle)
        self.targetpub.set(self.target_angle)
        self.pivot_currentpub.set(self.get_pivot_motor_current)
        self.intake_runningpub.set(self.intake_running)
        self.horizontalmotor_currentpub(self.get_horizontal_motor_current)



    

