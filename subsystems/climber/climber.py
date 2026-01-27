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
        self.leader_motor = hardware.TalonFX(constants.leader_motor_id)
        self.follower_motor = hardware.TalonFX(constants.follower_motor_id)
        self.motors = [self.follower_motor, self.leader_motor]


    def init(self) -> None:
        self.leader_motor_out = controls.VoltageOut(0)
        self.follower_motor_out = controls.VoltageOut(0)
        self.leader_motor.configurator.apply(constants.leader_motor_configs)
        self.follower_motor.configurator.apply(constants.follower_motor_configs)
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("climber")
        self.pos_pub = self.table.getDoubleTopic("climber_motor_revolutions").publish()
        self.moving_pub = self.table.getBooleanTopic("climber_moving").publish()
        self.zero_pub = self.table.getBooleanTopic("climber_zeroed").publish()
        self.current_pub = self.table.getDoubleTopic("climber_motor_current").publish() # supply current
        self.zero()

    def zero(self) -> None: 
        for motor in self.motors:
            motor.set_position(0)
        self.zeroed = True

    def get_motor_revolutions(self) -> float: 
        return self.leader_motor.get_position().value

    def set_position(self, target) -> None:
        if target > 0:
            for motor in self.motors:
                motor.set_control(controls.MotionMagicVoltage(target)) 
        else:
            for motor in self.motors:
                motor.set_control(controls.VoltageOut(constants.climber_drop_voltage))
        self.moving = True
        target = 0

    def get_motor_position(self):
        for motor in self.motors:
            return motor.get_position().value

    def update_table(self) -> None:
        self.pos_pub.set(self.get_motor_revolutions())
        self.moving_pub.set(self.moving)
        self.zero_pub.set(self.zeroed)
        self.current_pub.set(self.leader_motor.get_supply_current().value)

    def periodic(self) -> None:
        self.update_table()
