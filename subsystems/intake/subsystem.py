from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals, units
import math
import commands2
from .constants import *
import ntcore
from utils.phoenix_util import apply_config


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.drive_motor_left = hardware.TalonFX(drive_motor_left_id)
        self.drive_motor_right = hardware.TalonFX(drive_motor_right_id)

        self.slide_motor_left = hardware.TalonFX(slide_motor_left_id)
        self.slide_motor_right = hardware.TalonFX(slide_motor_right_id)

        self.drive_motor_out = controls.DutyCycleOut(0)
        self.slide_position_voltage = controls.PositionVoltage(0.0)
        self.slide_voltage = controls.VoltageOut(0)

        apply_config(self.drive_motor_left, drive_motor_configs)
        apply_config(self.drive_motor_right, drive_motor_configs)
        self.drive_motor_right.set_control(controls.Follower(drive_motor_left_id, signals.MotorAlignmentValue.OPPOSED))

        apply_config(self.slide_motor_left, slide_motor_configs)
        apply_config(self.slide_motor_right, slide_motor_configs.with_motor_output(
            configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
        ))
        # self.slide_motor_right.set_control(controls.Follower(slide_motor_left_id, signals.MotorAlignmentValue.OPPOSED))


        #initial zero
        self.slide_motor_left.set_position(intake_initial_position)
        self.slide_motor_right.set_position(intake_initial_position)
        
        self.target_position = 0
        self.intake_running = False
        self.pivot_running = False

        self.table = ntcore.NetworkTableInstance.getDefault().getTable("Intake")
        self.pospub = self.table.getDoubleTopic("slide position").publish()
        self.targetpub = self.table.getDoubleTopic("target position").publish()
        self.slide_supply_currentpub = self.table.getDoubleTopic("slide supply current").publish()
        self.drive_motor_currentpub = self.table.getDoubleTopic("drive motor current").publish()
        self.intake_runningpub = self.table.getBooleanTopic("intake running").publish()
        self.slide_stator_currentpub = self.table.getDoubleTopic("slide stator current").publish()

    def intake_fuel(self, speed=fuel_speed):
        """
        run intake
        """
        self.drive_motor_left.set_control(self.drive_motor_out.with_output(speed))
        self.intake_running = True

    def reverse_intake(self):
        """
        reverse intake
        """
        self.drive_motor_left.set_control(self.drive_motor_out.with_output(-fuel_speed))
        self.intake_running = True

    def stop_intake(self):
        """
        stop intake
        """
        self.drive_motor_left.set_control(self.drive_motor_out.with_output(0.0))
        self.intake_running = False

    def get_pivot_motor_supply_current(self):
        """
        get SUPPLY current of pivot motor
        """
        return self.slide_motor_left.get_supply_current()
    
    def get_pivot_motor_stator_current(self):
        """
        get STATOR current of pivot motor
        """
        return self.slide_motor_left.get_stator_current()
    
    def get_horizontal_motor_supply_current(self):
        """
        get SUPPLY current of horizontal motor
        """
        return self.drive_motor_left.get_supply_current()
    
    def get_pivot_angle(self) -> units.inches:
        """
        get rotations of pivot motor
        """
        return (self.slide_motor_left.get_position().value * slide_couple_ratio)
       
    def stop_pivot(self):
        """
        stop pivot motor
        """
        self.slide_motor_left.set_control(self.slide_voltage.with_output(0))
        self.slide_motor_right.set_control(self.slide_voltage.with_output(0))
        
    def set_slide_motor_voltage(self, output: units.volt):
        """
        set pivot motor voltage
        """
        self.output = output
        self.slide_motor_left.set_control(self.slide_voltage.with_output(self.output))
        self.slide_motor_right.set_control(self.slide_voltage.with_output(self.output))

    def zero_intake(self):
        self.slide_motor_left.set_position(intake_deploy_position/slide_couple_ratio)
        self.slide_motor_right.set_position(intake_deploy_position/slide_couple_ratio)
    
    def is_at_angle(self, angle: units.rotation):
        """
        checks at angle 
        """
        return abs(self.get_pivot_angle() - angle) < slide_threshold

    def set_pivot(self, pos: units.inches):
        """
        set pivot motor angle
        """
        self.target_position = self.limit_slide_pos(pos) / slide_couple_ratio
        self.slide_motor_left.set_control(self.slide_position_voltage.with_position(self.target_position))
        self.slide_motor_right.set_control(self.slide_position_voltage.with_position(self.target_position))

    def limit_slide_pos(self, pos: units.inches):
        """
        limit angle request to max pivot motor
        """
        if pos >= intake_deploy_position:
            return intake_deploy_position
        elif pos <= intake_initial_position:
            return intake_initial_position
        return pos
    
    def update_table(self):
        """
        update network tables
        """
        self.pospub.set(self.get_pivot_angle())
        self.targetpub.set(self.target_position)
        self.slide_supply_currentpub.set(self.get_pivot_motor_supply_current().value)
        self.intake_runningpub.set(self.intake_running)
        self.drive_motor_currentpub.set(self.get_horizontal_motor_supply_current().value)
        self.slide_stator_currentpub.set(self.get_pivot_motor_stator_current().value)

    def periodic(self):
        if NT_INTAKE:
            self.update_table()