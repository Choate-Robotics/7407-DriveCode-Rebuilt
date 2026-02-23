from unittest import signals
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
from phoenix6.signals import FeedbackSensorSourceValue
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration

# ids
horizontal_motor_id = 25 #placeholder
pivot_motor_id = 24 #placeholder

# constants
angle_threshold = 0.01 #placeholder 
fuel_speed = 0.75  #placeholder
voltage_out = 3 #placeholder

# 
intake_retract_rotation = 0.25
intake_deploy_rotation = 0
intake_maximum_rotation = 0.3527


horizontal_motor_configs = (
    configs.TalonFXConfiguration()
    .with_motor_output(
        configs.MotorOutputConfigs()
        .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
        .with_neutral_mode(signals.NeutralModeValue.BRAKE)
    ).with_current_limits(
        configs.CurrentLimitsConfigs()
        .with_stator_current_limit(80) #placeholder
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
                .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.ROTOR_SENSOR)
                .with_sensor_to_mechanism_ratio(45)
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
            ).with_current_limits(
                configs.CurrentLimitsConfigs()
                .with_stator_current_limit(60) #placeholder found experimentally
            )

        )