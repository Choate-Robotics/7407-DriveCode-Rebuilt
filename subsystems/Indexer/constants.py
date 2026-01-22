from phoenix6 import configs

#motor configs
indexer_motor_id = 30 #TODO: placeholder
tower_motor_id = 31 #TODO: placeholder

indexer_speed = 0 #TODO: placeholder
tower_speed = 0 #TODO: placeholder

indexer_config = configs.TalonFXConfiguation().with_motor_output(
    configs.MotorOutputConfigs()
    .with_neutral_mode(signals.NeutralModeValue.BRAKE)
    .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
)

tower_config = configs.TalonFXConfiguation().with_motor_output(
    configs.MotorOutputConfigs()
    .with_neutral_mode(signals.NeutralModeValue.BRAKE)
    .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
)

