from phoenix6 import hardware, configs, signals

left_lead_id = 58 # placeholder
left_follower_id = 59 # placeholder
right_lead_id = 60 # placeholder
right_follow_id = 61 # placeholder
hood_id = 62 # placeholder

flywheel_threshold = 2.0 # placeholder
hood_threshold = 2.0 # placeholder

hood_gear_ratio = 0 # placeholder

left_direction = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
right_direction = signals.InvertedValue.CLOCKWISE_POSITIVE

flywheel_config = configs.TalonFXConfiguration().with_motor_output(
    configs.MotorOutputConfigs()
    .with_neutral_mode(signals.NeutralModeValue.BRAKE)
).with_slot0(
    configs.Slot0Configs()
    .with_k_p(0) # placeholder
    .with_k_i(0) # placeholder
    .with_k_d(0) # placeholder
    .with_k_s(0) # placeholder
    .with_k_v(0) # placeholder
    .with_k_a(0) # placeholder
)
        
hood_config = configs.TalonFXConfiguration().with_motor_output(
    configs.MotorOutputConfigs()
    .with_neutral_mode(signals.NeutralModeValue.BRAKE)
    .with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
).with_motion_magic(
    configs.MotionMagicConfigs()
    .with_motion_magic_cruise_velocity(0) # placeholder
    .with_motion_magic_acceleration(0) # placeholder
    .with_motion_magic_jerk(0) # placeholder
).with_slot0(
    configs.Slot0Configs()
    .with_k_p(0) # placeholder
    .with_k_i(0) # placeholder
    .with_k_d(0) # placeholder
    .with_k_s(0) # placeholder
    .with_k_v(0) # placeholder
    .with_k_a(0) # placeholder
    .with_gravity_type(signals.GravityTypeValue.ARM_COSINE)
    .with_k_g(0) # placeholder
).with_feedback(
    configs.FeedbackConfigs()
    .with_sensor_to_mechanism_ratio(0) # placeholder
    .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.ROTOR_SENSOR)
)