import constants
from commands2 import Subsystem
from phoenix6 import hardware, controls, configs, signals

class Shooter(Subsystem):
    def __init__(self):
        super().__init__()
        self.left_leader_motor = hardware.TalonFX(constants.left_lead_id)
        self.left_follower_motor = hardware.TalonFX(constants.left_follower_id)

        self.right_leader_motor = hardware.TalonFX(constants.right_lead_id)
        self.right_follower_motor = hardware.TalonFX(constants.right_follow_id)

        self.hood_motor = hardware.TalonFX(constants.hood_id)
    
    def init(self):
        self.left_leader_motor.configurator.apply(constants.flywheel_config.with_motor_output(
            configs.MotorOutputConfigs()
            .with_inverted(constants.left_direction)
        ))
        self.left_follower_motor.set_control(controls.Follower(constants.left_lead_id, False))

        self.right_leader_motor.configurator.apply(constants.flywheel_config.with_motor_output(
            configs.MotorOutputConfigs()
            .with_inverted(constants.right_direction)
        ))
        self.right_follower_motor.set_control(controls.Follower(constants.right_lead_id, False))

        self.hood_motor.configurator.apply(constants.hood_config)