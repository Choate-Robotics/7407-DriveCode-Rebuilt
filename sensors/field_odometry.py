from .photonvision import PhotonController, PhotonCamCustom
from subsystems import CommandSwerveDrivetrain
from utils.field_constants import *
import robot_constants

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

        std_dev = 2

        distance_to_target = tags[0].getBestCameraToTarget().translation().toTranslation2d().distance(Translation2d(0, 0))

        total_tag_area = sum([tag.getArea() for tag in tags])

        if tag_count == 0:
            return

        if tag_count == 1:
            if distance_to_target > robot_constants.odometry_tag_distance:
                return
            for cam in self.cam_controller.cams:
                if cam.name == robot_constants.zoom_cam_name:
                    if 25 <= primary_id <= 26:
                        std_dev = 0.2 #placeholder
                    else:
                        std_dev = 0.3 #placeholder
                else:
                    std_dev =  0.4 #placeholder
        
        if tag_count == 2:
            std_dev = 0.7
            for cam in self.cam_controller.cams:
                if cam.name == robot_constants.zoom_cam_name:
                    if 25 <= primary_id <= 26:
                        std_dev = 0.1
                    else:
                        std_dev = 0.2
                else: 
                    if distance_to_target <= 0.75:
                        std_dev = 0.4

        if tag_count >= 3:
            std_dev = 0.4
            if 24 <= primary_id <= 27:
                std_dev = 0.1

        self.drivetrain.add_vision_measurement(Pose2d(), time, (std_dev,std_dev,std_dev))
                    


