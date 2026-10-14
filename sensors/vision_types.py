"""Shared NT4 struct definition. Keep this file IDENTICAL on the coprocessor and robot.

One VisionObservation = one camera's robot-pose estimate from one frame.
The coprocessor publishes a StructArray of these on a single topic, so the robot
does one NT read per loop no matter how many cameras there are.

Timestamping: the NT message timestamp is the publish time (NT4 converts it to the
robot's clock automatically). Each observation carries ageSeconds = publish time -
frame capture time, so the robot recovers capture time as:
    capture_time = message_time - ageSeconds
This stays correct even though each camera captures at a different instant.
"""

import dataclasses

import wpiutil.wpistruct as wpistruct
from wpimath.geometry import Pose3d


# The name is versioned: bump it whenever the fields change, so a robot and coprocessor
# on different versions see no data (type mismatch) instead of misreading each other's bytes.
@wpistruct.make_wpistruct(name="VisionObservationV3")
@dataclasses.dataclass
class VisionObservation:
    cameraId: wpistruct.int32  # index into the coprocessor's camera list
    robotPose: Pose3d  # field-relative robot pose (WPILib blue-origin coordinates)
    altRobotPose: Pose3d  # single tag: the other IPPE candidate (pick the one matching the
    #                       gyro heading); multi-tag: identical to robotPose
    ageSeconds: wpistruct.double  # publish time minus capture time
    stdDevFactor: wpistruct.double  # avgTagDistance^2 / tagCount^2 * camera std_dev_factor;
    #                                 the robot multiplies by its tunable coefficients
    tagCount: wpistruct.int32
    avgTagDistance: wpistruct.double  # meters, camera to tags
    ambiguity: wpistruct.double  # single-tag best_error / alt_error; 0 for multi-tag
    tagMask: wpistruct.int64  # bit i set = tag ID i was used (IDs 0-63)


def tag_ids_from_mask(mask: int) -> list[int]:
    return [i for i in range(64) if mask & (1 << i)]
