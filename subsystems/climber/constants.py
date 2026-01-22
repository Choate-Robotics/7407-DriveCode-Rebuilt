from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals
#ID values
leader_motor_id = 26 #placeholder
follower_motor_id = 27 #placeholder
climber_cancoder_id = 23 #placeholder

# other constants
climber_motion_magic_cruise_velocity = 97 #placeholder
climber_gear_ratio = 2 #placeholder
climber_drop_voltage = 2 #placeholder
motor_resistance = 2


leader_motor_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            ).with_motion_magic(
                configs.MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(climber_motion_magic_cruise_velocity)
                
            ).with_slot0(
                configs.Slot0Configs() #all placeholders
                .with_k_p(2)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(0.195)
                .with_k_v(0)
                .with_k_a(0)
            )
)


follower_motor_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)

            ).with_feedback(
                configs.FeedbackConfigs()
                .with_feedback_remote_sensor_id(signals.FeedbackSensorSourceValue.FUSED_CANCODER.value)
                .with_sensor_to_mechanism_ratio(climber_gear_ratio)
                
            ).with_motion_magic(
                configs.MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(climber_motion_magic_cruise_velocity) 
                
            ).with_slot0(
                configs.Slot0Configs() #all placeholders
                .with_k_p(2)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(0.195)
                .with_k_v(0)
                .with_k_a(0)
            )
)