
from .constants import *
from commands2 import Subsystem
from phoenix6 import hardware, controls
import ntcore
from utils.phoenix_util import apply_config

class Indexer(Subsystem):

    def __init__(self):
        super().__init__()
        self.indexer_motor: hardware.TalonFX = hardware.TalonFX(indexer_motor_id, "canivore")
        self.tower_motor: hardware.TalonFX = hardware.TalonFX(tower_motor_id)
        self.control_duty_cycle = controls.DutyCycleOut(0)
        self.control_velocity = controls.VelocityTorqueCurrentFOC(0)

        self.indexer_config = indexer_config
        self.tower_config = tower_config

        self.indexer_running: bool = False
        self.indexer_reversed: bool = False

        apply_config(self.indexer_motor, self.indexer_config)
        apply_config(self.tower_motor, self.tower_config)

        self.table = ntcore.NetworkTableInstance.getDefault().getTable("indexer")
        self.indexer_running_pub = self.table.getBooleanTopic("indexer running").publish()
        self.indexer_reversed_pub = self.table.getBooleanTopic("indexer reversed").publish()
        self.tower_motor_current_pub = self.table.getDoubleTopic("tower motor current").publish()
        self.indexer_motor_current_pub = self.table.getDoubleTopic("indexer motor current").publish()
        self.tower_motor_velocity_pub = self.table.getDoubleTopic("tower motor velocity").publish()
        self.indexer_motor_velocity_pub = self.table.getDoubleTopic("indexer motor velocity").publish()

    def run_indexer(self):
        """
        Runs the indexer motor
        """
        self.indexer_motor.set_control(
            self.control_duty_cycle.with_output(indexer_speed)
        )

        self.indexer_running = True
        self.indexer_reversed: bool = False

    def run_tower(self):
        """
        Runs the tower motor
        """
        self.tower_motor.set_control(
            self.control_duty_cycle.with_output(tower_duty_cycle)
        )

        self.indexer_running = True
        self.indexer_reversed: bool = False
        
    def run_indexer_reverse(self):
        """
        Runs the indexer motor in reverse
        """
        self.indexer_motor.set_control(
            self.control_duty_cycle.with_output(-indexer_speed_reversed)
        )

        self.indexer_running = True
        self.indexer_reversed: bool = True

    def run_tower_reverse(self):
        """
        Runs the tower motor in reverse
        """
        self.tower_motor.set_control(
            self.control_velocity.with_velocity(-tower_speed)
        )
        
        self.indexer_running = True
        self.indexer_reversed: bool = True

    def stop_indexer_motor(self):
        """
        Stops the indexer motor
        """
        self.indexer_motor.set_control(
            self.control_duty_cycle.with_output(0)
        )
        self.indexer_running = False
    
    def stop_tower_motor(self):
        """
        Stops the tower motor
        """
        self.tower_motor.set_control(
            self.control_duty_cycle.with_output(0)
        )
        self.indexer_running = False
    
    def get_tower_motor_current(self) -> float:
        """
        gets tower motor (supply) current
        """
        return self.tower_motor.get_supply_current().value

    def get_indexer_motor_current(self) -> float:
        """
        gets indexer motor (supply) current
        """
        return self.indexer_motor.get_supply_current().value
    

    def get_indexer_motor_velocity(self) -> float:
        """
        gets indexer motor velocity (rotations per second)
        """
        return self.indexer_motor.get_rotor_velocity().value
    
    def get_tower_motor_velocity(self) -> float:
        """
        gets tower motor velocity (rotations per second)
        """
        return self.tower_motor.get_rotor_velocity().value

    def update_table(self) -> None:
        """
        updates network tables
        """

        self.indexer_running_pub.set(self.indexer_running)
        # self.indexer_reversed_pub.set(self.indexer_reversed)
        self.tower_motor_current_pub.set(self.get_tower_motor_current())
        self.indexer_motor_current_pub.set(self.get_indexer_motor_current())
        self.indexer_motor_velocity_pub.set(self.get_indexer_motor_velocity())
        self.tower_motor_velocity_pub.set(self.get_tower_motor_velocity())

    def periodic(self):
        if NT_INDEXER:
            self.update_table()