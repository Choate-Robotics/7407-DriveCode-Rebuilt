from .photonvision import PhotonController
from subsystems import CommandSwerveDrivetrain
from utils.field_constants import *

from photonlibpy import EstimatedRobotPose

from wpimath.geometry import Translation2d

class FieldOdometry:
    def __init__(self, drivetrain: CommandSwerveDrivetrain, cam_controller: PhotonController):
        self.drivetrain = drivetrain
        self.cam_controller = cam_controller

        self.use_vision = True

    def enable(self):
        self.use_vision = True

    def disable(self):
        self.use_vision = False

    def add_vision_measure(self, estimated_pose: EstimatedRobotPose):
        pose = estimated_pose.estimatedPose.toPose2d()
        time = estimated_pose.timestampSeconds

        tags = estimated_pose.targetsUsed
        tag_count = len(tags)
        ids = [tag.fiducialId for tag in tags]
        primary_id = ids[0]

        distance_to_target = tags[0].getBestCameraToTarget().translation().toTranslation2d().distance(Translation2d(0, 0))

        total_tag_area = sum([tag.getArea() for tag in tags])