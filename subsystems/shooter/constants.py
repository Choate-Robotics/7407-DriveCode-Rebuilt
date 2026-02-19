from phoenix6 import hardware, configs, signals
from wpimath.geometry import Translation2d
import wpilib
import numpy as np

left_lead_id: int = 58 # placeholder
left_follower_id: int = 59 # placeholder
right_lead_id: int = 60 # placeholder
right_follow_id: int = 61 # placeholder
hood_id: int = 62 # placeholder
hood_cancoder_id: int = 63 #placeholder

flywheel_threshold = 2.0 # placeholder
hood_threshold = 2.0 # placeholder

hood_gear_ratio = 0 # placeholder

idle_velocity = 0 # placeholder

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
    .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.FUSED_CANCODER)  
    .with_feedback_remote_sensor_id(hood_cancoder_id)
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

max_hood_angle = 43 # placeholder
min_hood_angle = 0 # placeholder

# robot distance to hub, hood angle, and RPS
SHOT_TABLE = load_shooter_table_csv("shot_table.csv")

DIST_M = SHOT_TABLE[:, 0]
HOOD_DEG = SHOT_TABLE[:, 1]
RPS = SHOT_TABLE[:, 2]

# robot distance to pass, hood angle, and RPS
PASS_TABLE = load_shooter_table_csv("pass_table.csv")

PASS_DIST_M = SHOT_TABLE[:, 0]
PASS_HOOD_DEG = SHOT_TABLE[:, 1]
PASS_RPS = SHOT_TABLE[:, 2]