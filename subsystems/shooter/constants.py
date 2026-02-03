from phoenix6 import hardware, configs, signals
from wpimath.geometry import Translation2d
import numpy as np

left_lead_id = 58 # placeholder
left_follower_id = 59 # placeholder
right_lead_id = 60 # placeholder
right_follow_id = 61 # placeholder
hood_id = 62 # placeholder

flywheel_threshold = 2.0 # placeholder
hood_threshold = 2.0 # placeholder

hood_gear_ratio = 0 # placeholder

NT_SHOOTER: bool = True

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

shooter_offset = Translation2d(1, 1) # placeholder
max_hood_angle = 75 # placeholder

# robot distance to hub, hood angle, and RPS
SHOT_TABLE = np.array([
    [1.5, 22.0, 50],
    [2.0, 25.0, 55],
    [3.0, 30.0, 60],
    [4.0, 36.0, 65],
    [5.5, 43.0, 70],
    [6.5, 49.0, 75],
], dtype=float)

DIST_M = SHOT_TABLE[:, 0]
HOOD_DEG = SHOT_TABLE[:, 1]
RPM = SHOT_TABLE[:, 2]

# robot distance to pass, hood angle, and RPS
PASS_TABLE = np.array([
    [1.5, 22.0, 50],
    [2.0, 25.0, 55],
    [3.0, 30.0, 60],
    [4.0, 36.0, 65],
    [5.5, 43.0, 70],
    [6.5, 49.0, 75],
], dtype=float)


PASS_DIST_M = SHOT_TABLE[:, 0]
PASS_HOOD_DEG = SHOT_TABLE[:, 1]
PASS_RPM = SHOT_TABLE[:, 2]