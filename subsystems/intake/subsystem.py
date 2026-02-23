from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals, units
import math
import commands2
from .constants import *
import ntcore


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        # self.encoder: CANcoder = CANcoder()

        self.horizontal_motor = hardware.TalonFX(horizontal_motor_id)
        self.pivot_motor = hardware.TalonFX(pivot_motor_id)

        self.horizontal_motor_out = controls.DutyCycleOut(0)
        self.pivot_motion_magic = controls.MotionMagicVoltage(0.0)
        self.pivot_voltage = controls.VoltageOut(0)
        self.target_angle = 0.0    

        #initial zero
        self.pivot_motor.set_position(intake_deploy_rotation)
        
        self.intake_running = False
        self.pivot_running = False

        self.horizontal_motor.configurator.apply(horizontal_motor_configs)
        self.pivot_motor.configurator.apply(pivot_motor_configs)
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("Intake")
        self.anglepub = self.table.getDoubleTopic("pivot angle").publish()
        self.targetpub = self.table.getDoubleTopic("target angle").publish()
        self.pivot_supply_currentpub = self.table.getDoubleTopic("pivot supply current").publish()
        self.horizontal_motor_currentpub = self.table.getDoubleTopic("horizontal motor current").publish()
        self.intake_runningpub = self.table.getBooleanTopic("intake running").publish()
        self.pivot_stator_currentpub = self.table.getDoubleTopic("pivot stator current").publish()

    def intake_fuel(self):
        """
        run intake
        """
        self.horizontal_motor.set_control(self.horizontal_motor_out.with_output(fuel_speed))
        self.intake_running = True

    def reverse_intake(self):
        """
        reverse intake
        """
        self.horizontal_motor.set_control(self.horizontal_motor_out.with_output(-fuel_speed))
        self.intake_running = True

    def stop_intake(self):
        """
        stop intake
        """
        self.horizontal_motor.set_control(self.horizontal_motor_out.with_output(0.0))
        self.intake_running = False

    def get_pivot_motor_supply_current(self):
        """
        get SUPPLY current of pivot motor
        """
        return self.pivot_motor.get_supply_current()
    
    def get_pivot_motor_stator_current(self):
        """
        get STATOR current of pivot motor
        """
        return self.pivot_motor.get_stator_current()
    
    def get_horizontal_motor_supply_current(self):
        """
        get SUPPLY current of horizontal motor
        """
        return self.horizontal_motor.get_supply_current()
    
    def get_pivot_angle(self) -> units.rotation:
        """
        get rotations of pivot motor
        """
        return (self.pivot_motor.get_position().value)
       
    def stop_pivot(self):
        """
        stop pivot motor
        """
        self.pivot_motor.set_control(self.pivot_voltage.with_output(0))
        
    def set_pivot_motor_in(self, output: units.volt):
        """
        set pivot motor voltage
        """
        self.output = output
        self.pivot_motor.set_control(self.pivot_voltage.with_output(self.output))
    
    def is_at_angle(self, angle: units.rotation):
        """
        checks at angle 
        """
        return abs(self.get_pivot_angle() - angle) < angle_threshold

    def set_pivot(self, angle: units.rotation):
        """
        set pivot motor angle
        """
        self.target_angle = self.limit_pivot_angle(angle)
        self.pivot_motor.set_control(self.pivot_motion_magic.with_position(self.target_angle))

    def limit_pivot_angle(self, angle: units.rotation):
        """
        limit angle request to max pivot motor
        """
        if angle >= intake_maximum_rotation:
            return intake_maximum_rotation
        elif angle <= intake_retract_rotation:
            return intake_retract_rotation
        return angle
    
    def update_table(self):
        """
        update network tables
        """
        self.anglepub.set(self.get_pivot_angle())
        self.targetpub.set(self.target_angle)
        self.pivot_supply_currentpub.set(self.get_pivot_motor_supply_current().value)
        self.intake_runningpub.set(self.intake_running)
        self.horizontal_motor_currentpub.set(self.get_horizontal_motor_supply_current().value)
        self.pivot_stator_currentpub.set(self.get_pivot_motor_stator_current().value)

    def periodic(self):
        self.update_table()