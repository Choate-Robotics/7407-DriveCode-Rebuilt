from unittest import signals
from phoenix6 import configs, signals
import math
from wpimath import units

NT_INTAKE = True

# ids
drive_motor_left_id = 14
drive_motor_right_id = 15
slide_motor_left_id = 24
slide_motor_right_id = 25

# constants
slide_couple_ratio: units.inches = 1.25*math.pi

slide_threshold: units.inches = 0.01
fuel_speed = 1
voltage_out = 2
index_speed = 0
intake_index_time = 0.75 # seconds

intake_retract_position: units.inches = 0.5
intake_deploy_position: units.inches = 12.2
intake_initial_position: units.inches = 0


drive_motor_configs = (
    configs.TalonFXConfiguration()
    .with_motor_output(
        configs.MotorOutputConfigs()
        .with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
        .with_neutral_mode(signals.NeutralModeValue.COAST)
    ).with_current_limits(
        configs.CurrentLimitsConfigs()
        .with_stator_current_limit(80) #placeholder
    )
)

slide_motor_configs = (
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs()
                .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
                .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            ).with_feedback(
                configs.FeedbackConfigs()
                .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.ROTOR_SENSOR)
                .with_sensor_to_mechanism_ratio(45/12)
            ).with_slot0(
                configs.Slot0Configs()
                .with_k_p(6)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(0.45)
                .with_k_v(0)
                .with_k_a(0)
                .with_gravity_type(signals.GravityTypeValue.ELEVATOR_STATIC)
            ).with_current_limits(
                configs.CurrentLimitsConfigs()
                .with_stator_current_limit(60)
            )
        )