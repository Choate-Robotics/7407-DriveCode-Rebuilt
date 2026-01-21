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
       
        self.horizontal_motor_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
                
            )
        )

        self.pivot_motor_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            ).with_feedback(
                configs.FeedbackConfigs()
                .with_feedback_remote_sensor_id(signals.FeedbackSensorSourceValue.FUSED_CANCODER.value) 
                .with_sensor_to_mechanism_ratio(5) # placeholder
            ).with_motion_magic(
                configs.MotionMagicConfigs()
                # .with_motion_magic_cruise_velocity(0) placeholder
            )
        )
        self.pivot_motor_zeroed = False

    def init(self):
        self.horizontal_motor.configurator.apply(self.horizontal_motor_configs)
        self.pivot_motor.configurator.apply(self.pivot_motor_configs)

    def intake_fuel(self):
        self.horizontal_motor.set_control(controls.DutyCycleOut.with_output(constants.intake_fuel_speed))

    def get_pivot_motor_current(self):
        return self.pivot_motor.get_supply_current().value
    
    def zero_pivot(self):
        self.pivot_angle = (
            (self.encoder.get_absolute_position().value - constants.intake_encoder_zero) / (2 * math.pi))
        pos = self.pivot_angle
        self.pivot_motor.set_position(pos)
        self.pivot_motor_zeroed = True

    def get_pivot_angle(self):
        return (self.pivot_motor.get_position().value /
                (2
                * math.pi))
       
    def stop_pivot(self):
        self.pivot_motor.set_control(controls.DutyCycleOut.with_output(0.0))
        
    def is_at_angle(self):
        return abs(self.get_pivot_angle() - constants.intake_drop_angle) < constants.intake_angle_threshold

    def drop_intake(self):
        self.req = controls.VoltageOut(constants.intake_drop_voltage)
        self.pivot_motor.set_control(self.req)


