from phoenix6 import configs, signals

#motor configs
indexer_motor_id = 30 #TODO: placeholder
tower_motor_id = 31 #TODO: placeholder

indexer_speed = 0 #TODO: placeholder
tower_speed = 0 #TODO: placeholder

indexer_config = configs.TalonFXConfiguration().with_motor_output(
    configs.MotorOutputConfigs()
    .with_neutral_mode(signals.NeutralModeValue.BRAKE)
    .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
)

tower_config = configs.TalonFXConfiguration().with_motor_output(
    configs.MotorOutputConfigs()
    .with_neutral_mode(signals.NeutralModeValue.BRAKE)
    .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
)

#debouncer thresholds
motor_velocity_threshold = 0.3 #TODO: placeholder
motor_current_threshold = 0.4 #TODO: placeholder
debouncer_time = 1 #TODO: placeholder, in seconds

unjamming_time = 5 #number of runtime loops (of 20 ms)

#oi
trigger_threshold = 3 #TODO: placeholder