
import constants
from toolkit.subsystem import Subsystem
import wpilib
from phoenix6 import hardware, controls, configs, CANcoder, StatusCode
import ntcore

class Indexer(Subsystem):

    def __init__(self):
        super().__init__()
        self.indexer_motor: TalonFX = hardware.TalonFX(constants.indexer_motor_id)
        self.tower_motor: TalonFX = hardware.TalonFX(constants.tower_motor_id)

        self.indexer_config = constants.indexer_config
        self.tower_config = constants.tower_config

        self.indexer_running: bool = False
        self.indexer_reversed: bool = False

    def init(self):
        self.indexer_motor.configurator.apply(self.indexer_config)
        self.tower_motor.configurator.apply(self.tower_config)

    def run_indexer(self):

        self.indexer_motor.set_control(
            self.control.with_output(constants.indexer_speed)
        )

        self.indexer_running = True
        self.indexer_reversed: bool = False

    def run_tower(self):

        self.tower_motor.set_control(
            self.control.with_output(constants.tower_speed)
        )

        self.indexer_running = True
        self.indexer_reversed: bool = False
        
    def run_indexer_reverse(self):

        self.indexer_motor.set_control(
            self.control.with_output(-constants.indexer_speed)
        )

        self.indexer_running = True
        self.indexer_reversed: bool = True

    def run_tower_reverse(self):

        self.tower_motor.set_control(
            self.control.with_output(-constants.tower_speed)
        )
        
        self.indexer_running = True
        self.indexer_reversed: bool = True

    def stop(self):
        self.indexer_motor.set_control(
            self.control.with_output(0)
        )

        self.tower_motor.set_control(
            self.control.with_output(0)
        )

        self.indexer_running = False