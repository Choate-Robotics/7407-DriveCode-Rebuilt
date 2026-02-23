from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals
#ID values
left_motor_id = 26 #placeholder

# other constants
climber_gear_ratio = 125
climber_retract_voltage = 2 #placeholder
climber_deploy_speed = 2 #placeholder
climber_lower_bound = 0 #placeholder
climber_upper_bound = 30 #placeholder
L1_pos = 10 #placeholder

climber_motor_configs = (
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
                .with_motion_magic_cruise_velocity(97)
                
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