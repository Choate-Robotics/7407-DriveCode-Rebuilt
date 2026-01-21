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
            constants.TalonFXConfiguration()
            .with_motor_output(
                constants.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            )
        )

        self.pivot_motor_configs = (
            constants.TalonFXConfiguration()
            .with_motor_output(
                constants.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            ).with_feedback(
                constants.FeedbackConfigs()
                .with_feedback_remote_sensor_id(signals.FeedbackSensorSourceValue.FUSED_CANCODER)
            ).with_motion_magic(
                constants.MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(97)
            )
        )

    def init(self):
        self.horizontal_motor.configurator.apply(self.horizontal_motor_configs)
        self.horizontal_motor.init()
        self.pivot_motor.configurator.apply(self.pivot_motor_configs)
        self.pivot_motor.init()
        self.pivot_angle = (
            (self.encoder.get_absolute_position().value - constants.intake_encoder_zero) / (2 * math.pi))

    def get_pivot_angle(self):
        return self.pivot_angle.getposition().value
        
    def stop_pivot(self):
        self.pivot_motor.set_control(controls.DutyCycleOut.with_output(self,0))
        
    def is_at_angle(self, angle: float):
        return abs(self.get_pivot_angle()- angle) < constants.intake_angle_threshold)
        
    def 
    