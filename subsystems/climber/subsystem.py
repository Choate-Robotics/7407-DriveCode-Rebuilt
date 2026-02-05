import constants
from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
import commands2
import ntcore

class Climber(commands2.Subsystem):
    """
    Climber Class

    Has one motor that moves the climber.

    Methods:
        Zero method -> sets the current position as the zero

        set_position method -> sets the target position for the climber motors and moves them to said position. 
                               If that position is out of bounds, it will set to the nearest bound.

        set_voltage method -> sets the voltage for the climber motors

        get_motor_position -> returns the current positions of the left and right motors respectively
        
    The climber publishes its motor current, if it is zeroed, if it is moving, and the revolutions of the left motor to NetworkTables.
    
    The motor for the climber uses motion magic control for ascending and voltage control for descending. (defaulted to voltage out)
    """

    

    def __init__(self) -> None:
        super().__init__()
        self.moving = False
        self.zeroed = True

        self.motor = hardware.TalonFX(constants.left_motor_id)

        self.motor_out = controls.VoltageOut(0)
        self.climb_climber = controls.MotionMagicVoltage(0)
        self.drop_climber = controls.VoltageOut(0)

        self.motor.configurator.apply(constants.climber_motor_configs)

        self.setup_table()
        self.zero()

    def zero(self) -> None: 
        self.motor.set_position(0)
        self.zeroed = True

    def set_position(self, target) -> None:
        self.moving = True
        if target >= constants.climber_lower_bound and target <= constants.climber_upper_bound:
            self.motor.set_control(self.climb_climber.with_position(target))
        else:
            if target < constants.climber_lower_bound:
                target = constants.climber_lower_bound
            elif target > constants.climber_upper_bound:
                target = constants.climber_upper_bound
            self.motor.set_control(self.climb_climber.with_position(target))
        target = 0


    def set_voltage(self, voltage):
        self.motor.set_control(self.drop_climber.with_output(voltage))
        self.moving = True

    def get_motor_position(self):
        return self.motor.get_position().value
    
    def is_motor_position(self, position) -> bool:
        return (self.get_motor_position() >= position)

    def setup_table(self) -> None:
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("climber")
        self.pos_pub = self.table.getDoubleTopic("climber_motor_revolutions").publish()
        self.moving_pub = self.table.getBooleanTopic("climber_moving").publish()
        self.zero_pub = self.table.getBooleanTopic("climber_zeroed").publish()
        self.current_pub = self.table.getDoubleTopic("climber_motor_current").publish()

    def update_table(self) -> None:
        self.pos_pub.set(self.get_motor_position())
        self.moving_pub.set(self.moving)
        self.zero_pub.set(self.zeroed)
        self.current_pub.set(self.motor.get_supply_current().value)

    def periodic(self) -> None:
        self.update_table()
