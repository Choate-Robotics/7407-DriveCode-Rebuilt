from phoenix6 import hardware, configs, signals, units
from wpimath.geometry import Translation2d
import wpilib
import numpy as np

NT_SHOOTER: bool = True

left_lead_id: int = 16
left_follower_id: int = 17
right_lead_id: int = 18
right_follow_id: int = 19
hood_id: int = 20
hood_cancoder_id: int = 23

flywheel_velocity_threshold: units.rotations_per_second = 4.0
hood_angle_threshold: units.rotation = 2 / 360

hood_gear_ratio = 69 # 69:1
max_hood_angle: units.rotation = 50 / 360
min_hood_angle: units.rotation = 10 / 360

hood_clear_angle = 40 / 360

idle_velocity: units.rotations_per_second = 0
slow_velocity: units.rotations_per_second = 10

hood_cancoder_config = configs.CANcoderConfiguration().with_magnet_sensor(
    configs.MagnetSensorConfigs()
    .with_absolute_sensor_discontinuity_point(1)
    .with_magnet_offset(-0.0114)
    .with_sensor_direction(signals.SensorDirectionValue.CLOCKWISE_POSITIVE)
)

left_flywheel_config = configs.TalonFXConfiguration().with_motor_output(
    configs.MotorOutputConfigs()
    .with_neutral_mode(signals.NeutralModeValue.COAST)
    .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
).with_slot0(
    configs.Slot0Configs()
    .with_k_p(10)
    .with_k_i(0)
    .with_k_d(0)
    .with_k_s(6.25)
    .with_k_v(0.028)
    .with_k_a(0)
).with_current_limits(
    configs.CurrentLimitsConfigs()
    .with_stator_current_limit(80)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(80)
    .with_supply_current_limit_enable(True)
)

# right_flywheel_config = configs.TalonFXConfiguration().with_motor_output(
#     configs.MotorOutputConfigs()
#     .with_neutral_mode(signals.NeutralModeValue.COAST)
#     .with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
# ).with_slot0(
#     configs.Slot0Configs()
#     .with_k_p(15)
#     .with_k_i(0)
#     .with_k_d(0)
#     .with_k_s(8.5)
#     .with_k_v(0.095)
#     .with_k_a(0)
# ).with_current_limits(
#     configs.CurrentLimitsConfigs()
#     .with_stator_current_limit(80)
#     .with_stator_current_limit_enable(True)
#     .with_supply_current_limit(80)
#     .with_supply_current_limit_enable(True)
# )
        
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
    .with_k_p(125)
    .with_k_i(0)
    .with_k_d(1)
    .with_k_s(.5)
    .with_k_v(0)
    .with_k_a(0)
    .with_gravity_type(signals.GravityTypeValue.ARM_COSINE)
    .with_k_g(0)
).with_feedback(
    configs.FeedbackConfigs()
    .with_rotor_to_sensor_ratio(23*1.5)
    .with_sensor_to_mechanism_ratio(2)
    .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.FUSED_CANCODER)  
    .with_feedback_remote_sensor_id(hood_cancoder_id)
).with_current_limits(
    configs.CurrentLimitsConfigs()
    .with_stator_current_limit(60) # placeholder
)


def load_shooter_table_csv(rel_path: str) -> np.ndarray:
    """
    Loads a CSV from the robot deploy directory into a Nx3 float array:
    [distance_m, hood_deg, rps]
    """
    deploy_dir = wpilib.getDeployDirectory()
    file_path = deploy_dir + "/shooter_tables/" + rel_path

    # Load numeric rows, skip header
    table = np.loadtxt(file_path, delimiter=",", dtype=float)

    # Ensure shape is (N, 3) even if only one row
    table = np.atleast_2d(table)

    if table.shape[1] != 3:
        raise ValueError(f"Expected 3 columns (distance_m, hood_deg, rps), got {table.shape[1]} from {file_path}")
    
    return table

# robot distance to hub, hood angle, and RPS
SHOT_TABLE = load_shooter_table_csv("shot_table.csv")

DIST_M = SHOT_TABLE[:, 0]
HOOD_DEG = SHOT_TABLE[:, 2]
RPS = SHOT_TABLE[:, 1]

# robot distance to pass, hood angle, and RPS
PASS_TABLE = load_shooter_table_csv("pass_table.csv")

PASS_DIST_M = PASS_TABLE[:, 0]
PASS_HOOD_DEG = PASS_TABLE[:, 2]
PASS_RPS = PASS_TABLE[:, 1]