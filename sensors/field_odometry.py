from .photonvision import PhotonCamCustom
from subsystems import CommandSwerveDrivetrain
from utils.field_constants import *
import robot_constants

from photonlibpy import EstimatedRobotPose

from wpimath.geometry import Translation2d

class FieldOdometry:
    def __init__(self, drivetrain: CommandSwerveDrivetrain, cams: list[PhotonCamCustom]):
        self.drivetrain = drivetrain
        self.cams = cams

        self.use_vision = True

    def enable(self):
        self.use_vision = True

    def disable(self):
        self.use_vision = False

    def add_vision_measure(self, cam: PhotonCamCustom, estimated_pose: EstimatedRobotPose):
        self.cam_name = cam.name
        self.pose = estimated_pose.estimatedPose.toPose2d()
        self.time = estimated_pose.timestampSeconds

        tags = estimated_pose.targetsUsed
        tag_count = len(tags)

        std_dev = 2
        std_dev_rot = 10

        if tag_count == 0:
            return
        
        ids = [tag.fiducialId for tag in tags]
        primary_id = ids[0]
        distance_to_target = tags[0].getBestCameraToTarget().translation().toTranslation2d().distance(Translation2d(0, 0))

        total_tag_area = sum([tag.getArea() for tag in tags])

        if tag_count == 1:
            if distance_to_target > robot_constants.odometry_tag_distance:
                return

            if self.cam_name == robot_constants.zoom_cam_name:
                std_dev = 0.2 if 25 <= primary_id <= 26 else 0.3
            else:
                std_dev = 0.4

        elif tag_count == 2:
            std_dev = 0.7
            if self.cam_name == robot_constants.zoom_cam_name:
                std_dev = 0.1 if 25 <= primary_id <= 26 else 0.2
            else:
                if distance_to_target <= 0.75:
                    std_dev = 0.4

        else:  # tag_count >= 3
            std_dev = 0.4
            if 24 <= primary_id <= 27:
                std_dev = 0.1

        self.drivetrain.add_vision_measurement(self.pose, self.time, (std_dev, std_dev, std_dev_rot))

    def update(self):
        if not self.use_vision:
            return

        for cam in self.cams:
            est = cam.get_result()
            if est is not None:
                self.add_vision_measure(cam, est)
                cam.update_tables
                    


