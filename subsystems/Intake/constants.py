from unittest import signals
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
from phoenix6.signals import FeedbackSensorSourceValue
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration

horizontal_motor_id = 25 #placeholder
pivot_motor_id = 24 #placeholder
cancoder_id = 26 #placeholder
angle_threshold = 2 #placeholder 
fuel_speed = 0.0  #placeholder
deploy_angle = 45 #placeholder
initial_angle = 0 #placeholder
voltage_out = 3 #placeholder

horizontal_motor_configs = (
    configs.TalonFXConfiguration()
    .with_motor_output(
        configs.MotorOutputConfigs()
        .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
        .with_neutral_mode(signals.NeutralModeValue.BRAKE)
                
        )
)

pivot_motor_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            ).with_feedback(
                configs.FeedbackConfigs()
                .with_feedback_remote_sensor_id(pivot_motor_id)
                .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.FUSED_CANCODER)
                .with_sensor_to_mechanism_ratio(5) # placeholder
            ).with_motion_magic(
                configs.MotionMagicConfigs()
                # .with_motion_magic_cruise_velocity(0) placeholder
            ).with_slot0(
                configs.Slot0Configs()
                .with_k_p(0.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0)
                .with_k_a(0)
                .with_gravity_type(signals.GravityTypeValue.ARM_COSINE)
            ).with_software_limit_switch(
                configs.SoftwareLimitSwitchConfigs()
                .with_reverse_soft_limit_enable(True)
                .with_reverse_soft_limit_threshold(0.1) #placeholder
                .with_forward_soft_limit_enable(True)
                .with_forward_soft_limit_threshold(0.2) #placeholder
            ).with_current_limits(
                configs.CurrentLimitsConfigs()
                .with_stator_current_limit(0.4) #placeholder found experimentally
            )

        )