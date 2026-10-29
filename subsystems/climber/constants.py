from phoenix6 import configs, signals
left_motor_id = 2

climber_motor_configs = (
    configs.TalonFXConfiguration().with_motor_output(
        configs.MotorOutputConfigs().with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    )
)

climber_extend_voltage = 12
climber_upper_bound = 220