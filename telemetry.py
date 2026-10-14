import time

from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger, swerve, units
from wpilib import Color, Color8Bit, Field2d, Mechanism2d, MechanismLigament2d, RobotBase, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState


class Telemetry:
    _PUBLISH_EVERY_KEY = "Telemetry/Publish Every N Odometry Updates"
    _PUBLISH_EVERY_DEFAULT = 5

    def __init__(self, max_speed: units.meters_per_second):
        """
        Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed
        :type max_speed: units.meters_per_second
        """
        self._max_speed = max_speed
        # Latest chassis speeds from the odometry thread, for code that only needs a
        # recent value (vision std dev scaling) and shouldn't pay for a get_state() copy.
        self.latest_speeds = ChassisSpeeds()
        # Hoot logs only on the real robot. In sim they land in logs/ inside the project,
        # and robotpy deploy uploads .hoot files (it only skips .wpilog), so they block
        # deploys with "large files found".
        if RobotBase.isReal():
            SignalLogger.start()

        # What to publish over networktables for telemetry
        self._inst = NetworkTableInstance.getDefault()

        # Robot swerve drive state
        self._drive_state_table = self._inst.getTable("DriveState")
        self._drive_pose = self._drive_state_table.getStructTopic("Pose", Pose2d).publish()
        self._drive_speeds = self._drive_state_table.getStructTopic("Speeds", ChassisSpeeds).publish()
        self._drive_module_states = self._drive_state_table.getStructArrayTopic("ModuleStates", SwerveModuleState).publish()
        self._drive_module_targets = self._drive_state_table.getStructArrayTopic("ModuleTargets", SwerveModuleState).publish()
        self._drive_module_positions = self._drive_state_table.getStructArrayTopic("ModulePositions", SwerveModulePosition).publish()
        self._drive_timestamp = self._drive_state_table.getDoubleTopic("Timestamp").publish()
        self._drive_odometry_frequency = self._drive_state_table.getDoubleTopic("OdometryFrequency").publish()

        # Field2d for Elastic's Field widget (AdvantageScope can use DriveState/Pose directly)
        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)

        # Phoenix calls telemeterize() on its odometry thread after every odometry update
        # (up to 250 Hz). Publishing on every one is mostly wasted - NT only sends about
        # every 100 ms - and it's Python time shared with the main loop and the odometry
        # thread itself, so only every Nth update publishes. Odometry still runs at full
        # rate. Live-tunable for A/B testing; 1 = every update (the old behavior).
        SmartDashboard.setDefaultNumber(self._PUBLISH_EVERY_KEY, self._PUBLISH_EVERY_DEFAULT)
        self._publish_every = self._PUBLISH_EVERY_DEFAULT
        self._calls = 0

        # Timing/Telemetry Load % = share of wall time spent inside telemeterize(), i.e.
        # time no other Python (the main loop) can run. Updated once a second.
        self._busy_s = 0.0
        self._window_start = time.perf_counter()
        timing_table = self._inst.getTable("Timing")
        self._load_pub = timing_table.getDoubleTopic("Telemetry Load %").publish()
        self._calls_pub = timing_table.getDoubleTopic("Telemetry Calls per s").publish()

        # Robot pose for field positioning
        # self._table = self._inst.getTable("Pose")
        # self._field_pub = self._table.getDoubleArrayTopic("robotPose").publish()
        # self._field_type_pub = self._table.getStringTopic(".type").publish()

        # Mechanisms to represent the swerve module states
        # self._module_mechanisms: list[Mechanism2d] = [
        #     Mechanism2d(1, 1),
        #     Mechanism2d(1, 1),
        #     Mechanism2d(1, 1),
        #     Mechanism2d(1, 1),
        # ]
        # A direction and length changing ligament for speed representation
        # self._module_speeds: list[MechanismLigament2d] = [
        #     self._module_mechanisms[0]
        #     .getRoot("RootSpeed", 0.5, 0.5)
        #     .appendLigament("Speed", 0.5, 0),
        #     self._module_mechanisms[1]
        #     .getRoot("RootSpeed", 0.5, 0.5)
        #     .appendLigament("Speed", 0.5, 0),
        #     self._module_mechanisms[2]
        #     .getRoot("RootSpeed", 0.5, 0.5)
        #     .appendLigament("Speed", 0.5, 0),
        #     self._module_mechanisms[3]
        #     .getRoot("RootSpeed", 0.5, 0.5)
        #     .appendLigament("Speed", 0.5, 0),
        # ]
        # A direction changing and length constant ligament for module direction
        # self._module_directions: list[MechanismLigament2d] = [
        #     self._module_mechanisms[0]
        #     .getRoot("RootDirection", 0.5, 0.5)
        #     .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
        #     self._module_mechanisms[1]
        #     .getRoot("RootDirection", 0.5, 0.5)
        #     .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
        #     self._module_mechanisms[2]
        #     .getRoot("RootDirection", 0.5, 0.5)
        #     .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
        #     self._module_mechanisms[3]
        #     .getRoot("RootDirection", 0.5, 0.5)
        #     .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
        # ]

        # Set up the module state Mechanism2d telemetry
        # for i, module_mechanism in enumerate(self._module_mechanisms):
        #     SmartDashboard.putData(f"Module {i}", module_mechanism)

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Called by Phoenix after every odometry update; publishes every Nth one.
        """
        start = time.perf_counter()
        self.latest_speeds = state.speeds
        self._calls += 1
        if self._calls % self._publish_every == 0:
            self._publish(state)

        now = time.perf_counter()
        self._busy_s += now - start
        elapsed = now - self._window_start
        if elapsed >= 1.0:
            self._load_pub.set(100.0 * self._busy_s / elapsed)
            self._calls_pub.set(self._calls / elapsed)
            self._publish_every = max(1, int(SmartDashboard.getNumber(
                self._PUBLISH_EVERY_KEY, self._PUBLISH_EVERY_DEFAULT)))
            self._calls = 0
            self._busy_s = 0.0
            self._window_start = now

    def _publish(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to NetworkTables (logged to the .wpilog).
        """
        # Telemeterize the swerve drive state
        self._drive_pose.set(state.pose)
        self._field.setRobotPose(state.pose)
        self._drive_speeds.set(state.speeds)
        self._drive_module_states.set(state.module_states)
        self._drive_module_targets.set(state.module_targets)
        self._drive_module_positions.set(state.module_positions)
        self._drive_timestamp.set(state.timestamp)
        self._drive_odometry_frequency.set(1.0 / state.odometry_period)

        # No separate SignalLogger copy: DataLogManager already records these NT topics
        # into the .wpilog at the same rate, and the .hoot has the raw module and gyro
        # signals. Writing both was ~40% of this function's time.

        # Telemeterize the pose to a Field2d
        # self._field_type_pub.set("Field2d")

        # pose_array = [state.pose.x, state.pose.y, state.pose.rotation().degrees()]
        # self._field_pub.set(pose_array)

        # Telemeterize each module state to a Mechanism2d
        # for i, module_state in enumerate(state.module_states):
        #     self._module_speeds[i].setAngle(module_state.angle.degrees())
        #     self._module_directions[i].setAngle(module_state.angle.degrees())
        #     self._module_speeds[i].setLength(module_state.speed / (2 * self._max_speed))
