import constants
from phoenix6.hardware import CANcoder
from phoenix6 import StatusSignal, controls, configs, hardware, signals 
import commands2
import ntcore

#assuming the arms are not connected, but we want them to move with eachother
class Climber(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.moving = False
        self.zeroed = True
        self.left_motor = hardware.TalonFX(constants.leader_motor_id)
        self.right_motor = hardware.TalonFX(constants.follower_motor_id)
        self.motors = [self.right_motor, self.left_motor]
        self.left_motor_out = controls.VoltageOut(0)
        self.right_motor_out = controls.VoltageOut(0)


    def init(self) -> None:
        self.left_motor.configurator.apply(constants.climber_motor_configs)
        self.right_motor.configurator.apply(constants.climber_motor_configs)
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("climber")
        self.pos_pub = self.table.getDoubleTopic("climber_motor_revolutions").publish()
        self.moving_pub = self.table.getBooleanTopic("climber_moving").publish()
        self.zero_pub = self.table.getBooleanTopic("climber_zeroed").publish()
        self.current_pub = self.table.getDoubleTopic("climber_motor_current").publish() # supply current
        self.climb_climber = controls.MotionMagicVoltage(0)
        self.drop_climber = controls.VoltageOut(0)
        self.zero()

    def zero(self) -> None: 
        for motor in self.motors:
            motor.set_position(0)
        self.zeroed = True

    def get_motor_revolutions(self) -> float: 
        return self.left_motor.get_position().value

    def set_position(self, target) -> None:
        if target > 0:
            for motor in self.motors:
                motor.set_control(self.climb_climber.with_position(target)) 
        self.moving = True
        target = 0

    def set_voltage(self, voltage):
        for motor in self.motors:
                motor.set_control(self.drop_climber.with_output(voltage))
        self.moving = True

    def get_left_motor_position(self):
        return self.left_motor.get_position().value
    
    def get_right_motor_position(self):
        return self.right_motor.get_position().value

    def update_table(self) -> None:
        self.pos_pub.set(self.get_motor_revolutions())
        self.moving_pub.set(self.moving)
        self.zero_pub.set(self.zeroed)
        self.current_pub.set(self.left_motor.get_supply_current().value)

    def periodic(self) -> None:
        self.update_table()
