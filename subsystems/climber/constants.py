from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals

NT_CLIMBER = True

#ID values
left_motor_id = 24

# other constants
climber_retract_voltage = -9 #placeholder
climber_extend_voltage = 12 #placeholder
climber_lower_bound = 0
climber_upper_bound = 220
climb_l1 = 15

climber_motor_configs = (
    configs.TalonFXConfiguration()
    .with_motor_output(
        configs.MotorOutputConfigs()
        .with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
        .with_neutral_mode(signals.NeutralModeValue.BRAKE)
    )
)