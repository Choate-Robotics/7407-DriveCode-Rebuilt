
import constants
from commands2.Subsystem import Subsystem
from phoenix6 import hardware, controls
import ntcore

class Indexer(Subsystem):

    def __init__(self):
        super().__init__()
        self.indexer_motor: TalonFX = hardware.TalonFX(constants.indexer_motor_id)
        self.tower_motor: TalonFX = hardware.TalonFX(constants.tower_motor_id)
        self.control = controls.DutyCycleOut(0)

        self.indexer_config = constants.indexer_config
        self.tower_config = constants.tower_config

        self.indexer_running: bool = False
        self.indexer_reversed: bool = False

    def init(self):
        self.indexer_motor.configurator.apply(self.indexer_config)
        self.tower_motor.configurator.apply(self.tower_config)

        self.table = ntcore.NetworkTableInstance.getDefault().getTable("indexer")
        self.indexer_running_pub = self.table.getBooleanTopic("indexer running").publish()
        self.tower_motor_current_pub = self.table.getDoubleTopic("tower motor current").publish()
        self.indexer_motor_current_pub = self.table.getDoubleTopic("indexer motor current").publish()
        self.tower_motor_velocity_pub = self.table.getDoubleTopic("tower motor velocity").publish()
        self.indexer_motor_velocity_pub = self.table.getDoubleTopic("indexer motor velocity").publish()

    def run_indexer(self):
        """
        Runs the indexer motor
        """
        self.indexer_motor.set_control(
            self.control.with_output(constants.indexer_speed)
        )

        self.indexer_running = True
        self.indexer_reversed: bool = False

    def run_tower(self):
        """
        Runs the tower motor
        """
        self.tower_motor.set_control(
            self.control.with_output(constants.tower_speed)
        )

        self.indexer_running = True
        self.indexer_reversed: bool = False
        
    def run_indexer_reverse(self):
        """
        Runs the indexer motor in reverse
        """
        self.indexer_motor.set_control(
            self.control.with_output(-constants.indexer_speed)
        )

        self.indexer_running = True
        self.indexer_reversed: bool = True

    def run_tower_reverse(self):
        """
        Runs the tower motor in reverse
        """
        self.tower_motor.set_control(
            self.control.with_output(-constants.tower_speed)
        )
        
        self.indexer_running = True
        self.indexer_reversed: bool = True

    def stop_indexer_motor(self):
        """
        Stops the indexer motor
        """
        self.indexer_motor.set_control(
            self.control.with_output(0)
        )
        self.indexer_running = False
    
    def stop_tower_motor(self):
        """
        Stops the tower motor
        """
        self.tower_motor.set_control(
            self.control.with_output(0)
        )
        self.indexer_running = False
    
    def get_tower_motor_current(self) -> float:
        """
        gets tower motor (supply) current
        """
        return self.tower_motor.get_supply_current()

    def get_indexer_motor_current(self) -> float:
        """
        gets indexer motor (supply) current
        """
        return self.indexer_motor.get_supply_current()
    

    def get_indexer_motor_velocity(self) -> float:
        """
        gets indexer motor velocity (Volts)
        """
        return self.indexer_motor.get_rotor_velocity().Volts
    
    def get_tower_motor_velocity(self) -> float:
        """
        gets tower motor velocity (Volts)
        """
        return self.tower_motor.get_rotor_velocity().Volts

    def update_table(self) -> None:
        """
        updates network tables
        """
        table = ntcore.NetworkTableInstance.getDefault().getTable("Indexer")

        self.indexer_running_pub.set(self.indexer_running)
        self.indexer_reversed_pub.set(self.indexer_reversed)
        self.tower_motor_current_pub.set(self.get_tower_motor_current)
        self.indexer_motor_current_pub.set(self.get_indexer_motor_current)
        self.indexer_motor_velocity_pub.set(self.get_indexer_motor_velocity)
        self.tower_motor_velocity_pub.set(self.get_tower_motor_velocity)
