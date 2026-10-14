"""Drop-in FieldOdometry for 7407-DriveCode-Rebuilt, fed by the batched coprocessor
instead of PhotonVision.

Copy into sensors/ with batched_vision.py and vision_types.py. Construct it with a
BatchedVisionSource instead of the camera list:

    self.vision = BatchedVisionSource()
    self.field_odometry = FieldOdometry(self.drivetrain, self.vision, self.backup_gyro)

Std devs: xy = max(Min XY Std Dev, XY coefficient * stdDevFactor) * speed scale, where
the coprocessor computes stdDevFactor = avgTagDistance^2 / tagCount^2 * that camera's
std_dev_factor, and speed scale = 1 + speed / Speed Ref + turn rate / Turn Rate Ref.
The values are WPILib Preferences: tune them in Elastic under /Preferences/Vision/,
and they persist on the roboRIO across reboots and deploys. To trust one camera less
than the other, raise its std_dev_factor on the coprocessor's settings page instead
of special-casing it here.

Defaults are from the 2026-10-06 test logs (enabled, tags within ~3 m): multi-tag
error was ~2 cm per axis still or slow, ~4-5 cm above 1.5 m/s or turning hard, and
mostly a steady per-camera offset (1-4 cm) rather than frame-to-frame noise (0.2 cm),
so the std devs sit ~2x above the typical error. Single-tag factors are 4-12, so the
same coefficient gives them 30-80 cm, matching their worst readings.

Single tags: the coprocessor sends both candidate poses; select_pose() uses the one
whose heading matches the gyro and, while enabled, drops it if it's still more than
"Single Tag Max Yaw Error Deg" off. poll() also handles ignoring the first 2 s of
auto and asking the coprocessor to throttle while disabled (see batched_vision.py).
"""

import math
import time
from typing import Callable

from phoenix6.hardware import Pigeon2
from wpilib import DriverStation, Preferences, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds

import robot_constants
from subsystems import CommandSwerveDrivetrain

from .batched_vision import BatchedVisionSource, select_pose
from .vision_types import VisionObservation


class FieldOdometry:
    XY_COEFF_KEY = "Vision/XY Std Dev Coeff"
    THETA_COEFF_KEY = "Vision/Theta Std Dev Coeff"
    SINGLE_TAG_THETA_KEY = "Vision/Single Tag Theta Std Dev"
    SINGLE_TAG_MAX_YAW_ERR_KEY = "Vision/Single Tag Max Yaw Error Deg"
    MIN_XY_KEY = "Vision/Min XY Std Dev"
    SPEED_REF_KEY = "Vision/Speed Ref m per s"
    TURN_RATE_REF_KEY = "Vision/Turn Rate Ref rad per s"

    XY_COEFF_DEFAULT = 0.07  # ~3-14 cm multi-tag, ~30-80 cm single-tag
    THETA_COEFF_DEFAULT = 0.1  # ~6 deg at factor 1: multi-tag vision gently corrects heading
    SINGLE_TAG_THETA_DEFAULT = 100.0  # effectively "gyro owns heading"
    SINGLE_TAG_MAX_YAW_ERR_DEFAULT = 5.0
    MIN_XY_DEFAULT = 0.03  # close tags have tiny factors; don't trust them past their ~2 cm offsets
    SPEED_REF_DEFAULT = 1.5  # error roughly doubles by 1.5 m/s...
    TURN_RATE_REF_DEFAULT = 1.5  # ...or 1.5 rad/s

    # Earlier defaults. A stored Preference still equal to one of these was never tuned,
    # so it's moved to the new default once; anything tuned by hand is left alone.
    _OLD_DEFAULTS = {XY_COEFF_KEY: 0.2, THETA_COEFF_KEY: 5.0}

    def __init__(self, drivetrain: CommandSwerveDrivetrain, vision: BatchedVisionSource, backup_gyro: Pigeon2,
                 speeds_supplier: Callable[[], ChassisSpeeds] | None = None):
        """speeds_supplier: recent chassis speeds for std dev scaling (e.g. the telemetry
        callback's latest). Defaults to drivetrain.get_speeds(), which costs a state copy."""
        self.drivetrain = drivetrain
        self._speeds = speeds_supplier or drivetrain.get_speeds
        self.vision = vision
        self.backup_gyro = backup_gyro

        self.use_vision = True

        # init only writes a value if the key doesn't exist yet, so values tuned in
        # Elastic survive code deploys.
        Preferences.initDouble(self.XY_COEFF_KEY, self.XY_COEFF_DEFAULT)
        Preferences.initDouble(self.THETA_COEFF_KEY, self.THETA_COEFF_DEFAULT)
        Preferences.initDouble(self.SINGLE_TAG_THETA_KEY, self.SINGLE_TAG_THETA_DEFAULT)
        Preferences.initDouble(self.SINGLE_TAG_MAX_YAW_ERR_KEY, self.SINGLE_TAG_MAX_YAW_ERR_DEFAULT)
        Preferences.initDouble(self.MIN_XY_KEY, self.MIN_XY_DEFAULT)
        Preferences.initDouble(self.SPEED_REF_KEY, self.SPEED_REF_DEFAULT)
        Preferences.initDouble(self.TURN_RATE_REF_KEY, self.TURN_RATE_REF_DEFAULT)
        new_defaults = {self.XY_COEFF_KEY: self.XY_COEFF_DEFAULT, self.THETA_COEFF_KEY: self.THETA_COEFF_DEFAULT}
        for key, old in self._OLD_DEFAULTS.items():
            if Preferences.getDouble(key, old) == old:
                Preferences.setDouble(key, new_defaults[key])

        self._speed_scale = 1.0

        # Preferences are read once a second instead of several times per observation;
        # a value changed in Elastic takes effect within a second.
        self._pref_defaults = {
            self.XY_COEFF_KEY: self.XY_COEFF_DEFAULT,
            self.THETA_COEFF_KEY: self.THETA_COEFF_DEFAULT,
            self.SINGLE_TAG_THETA_KEY: self.SINGLE_TAG_THETA_DEFAULT,
            self.SINGLE_TAG_MAX_YAW_ERR_KEY: self.SINGLE_TAG_MAX_YAW_ERR_DEFAULT,
            self.MIN_XY_KEY: self.MIN_XY_DEFAULT,
            self.SPEED_REF_KEY: self.SPEED_REF_DEFAULT,
            self.TURN_RATE_REF_KEY: self.TURN_RATE_REF_DEFAULT,
        }
        self._prefs: dict[str, float] = {}
        self._prefs_read_at = -math.inf
        self._refresh_prefs()

    def _refresh_prefs(self):
        now = time.monotonic()
        if now - self._prefs_read_at >= 1.0:
            self._prefs_read_at = now
            self._prefs = {k: Preferences.getDouble(k, d) for k, d in self._pref_defaults.items()}

    def enable(self):
        self.use_vision = True

    def disable(self):
        self.use_vision = False

    def _update_speed_scale(self):
        """Vision error grows when driving fast or turning hard (blur, timing); scale up."""
        speeds = self._speeds()
        speed_ref = max(self._prefs[self.SPEED_REF_KEY], 1e-3)
        turn_ref = max(self._prefs[self.TURN_RATE_REF_KEY], 1e-3)
        self._speed_scale = 1.0 + math.hypot(speeds.vx, speeds.vy) / speed_ref + abs(speeds.omega) / turn_ref

    def std_devs(self, obs: VisionObservation) -> tuple[float, float, float]:
        xy = max(self._prefs[self.MIN_XY_KEY],
                 self._prefs[self.XY_COEFF_KEY] * obs.stdDevFactor)
        if obs.tagCount == 1:
            theta = self._prefs[self.SINGLE_TAG_THETA_KEY]
        else:
            theta = self._prefs[self.THETA_COEFF_KEY] * obs.stdDevFactor * self._speed_scale
        xy *= self._speed_scale
        return xy, xy, theta

    def add_vision_measure(self, capture_time: float, obs: VisionObservation, heading: Rotation2d):
        if obs.tagCount == 0:
            return

        if obs.tagCount == 1 and (
            obs.avgTagDistance > robot_constants.odometry_tag_distance
            or obs.ambiguity > getattr(robot_constants, "odometry_single_tag_ambiguity", 0.4)
        ):
            return

        max_yaw_err = None  # while disabled, accept everything so vision can seed the pose
        if DriverStation.isEnabled():
            max_yaw_err = math.radians(self._prefs[self.SINGLE_TAG_MAX_YAW_ERR_KEY])
        pose = select_pose(obs, heading, max_yaw_err)
        if pose is None:
            return

        self.drivetrain.add_vision_measurement(pose, capture_time, self.std_devs(obs))

    def update(self):
        SmartDashboard.putBoolean("Vision/Coprocessor Connected", self.vision.is_connected())

        self._refresh_prefs()

        # Always drain the queue, even while disabled, so re-enabling doesn't replay
        # a backlog of stale observations.
        observations = self.vision.poll()
        SmartDashboard.putBoolean("Vision/Sees Tag", self.vision.sees_tag())
        if not self.use_vision:
            return

        # Only the newest observation from each camera. Normally that's all there is
        # (~1 per camera per loop), but after a slow loop a backlog builds up, and feeding
        # all of it makes the next loop slow too. Older frames add little on top of the
        # newest one from the same camera.
        newest = {}
        for capture_time, obs in observations:  # oldest first, so later ones win
            newest[obs.cameraId] = (capture_time, obs)
        observations = sorted(newest.values(), key=lambda p: p[0])

        # Heading for single-tag pose selection, only when a single-tag observation needs it
        # (get_pose() shares one drivetrain state read per loop).
        heading = None
        if any(obs.tagCount == 1 for _, obs in observations):
            heading = self.drivetrain.get_pose().rotation()
        if observations:
            self._update_speed_scale()

        for capture_time, obs in observations:
            self.add_vision_measure(capture_time, obs, heading)

        if self.drivetrain.gyro_broken:
            self.drivetrain.reset_rotation(Rotation2d(math.radians(self.backup_gyro.get_yaw().value)))
