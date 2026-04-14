from phoenix6 import configs, signals

NT_INDEXER = True

#motor configs
indexer_motor_id = 22
tower_motor_id = 21

indexer_speed = 1
indexer_velocity = 85
tower_speed = 30
tower_duty_cycle = 1

indexer_config = configs.TalonFXConfiguration().with_motor_output(
    configs.MotorOutputConfigs()
    .with_neutral_mode(signals.NeutralModeValue.COAST)
    .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
).with_current_limits(
    configs.CurrentLimitsConfigs()
    .with_stator_current_limit(60)
).with_slot0(
    configs.Slot0Configs()
    .with_k_p(12)
    .with_k_i(0)
    .with_k_d(0)
    .with_k_s(14)
    .with_k_v(0.035)
    .with_k_a(0)
)

tower_config = configs.TalonFXConfiguration().with_motor_output(
    configs.MotorOutputConfigs()
    .with_neutral_mode(signals.NeutralModeValue.COAST)
    .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
).with_feedback(
     configs.FeedbackConfigs()
     .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.ROTOR_SENSOR)
     .with_sensor_to_mechanism_ratio(3) 
).with_slot0(
    configs.Slot0Configs()
    .with_k_p(17)
    .with_k_i(0)
    .with_k_d(0)
    .with_k_s(10)
    .with_k_v(0.22)
    .with_k_a(0)
).with_current_limits(
    configs.CurrentLimitsConfigs()
    .with_stator_current_limit(60)
)

#debouncer thresholds
motor_velocity_threshold = 0.3 #TODO: placeholder
motor_current_threshold = 0.4 #TODO: placeholder
debouncer_time = 1 #TODO: placeholder, in seconds

unjamming_time = 5 #number of runtime loops (of 20 ms)

#oi
trigger_threshold = 3 #TODO: placeholder