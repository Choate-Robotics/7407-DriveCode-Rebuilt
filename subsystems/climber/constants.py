from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
leader_motor_id = 123123231312123 #placeholder
follower_motor_id = 3132132123312 # placeholder

leader_motor_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            )
        )