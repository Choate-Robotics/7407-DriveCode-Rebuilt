from .photonvision import PhotonCamCustom
from subsystems import CommandSwerveDrivetrain
from wpilib import Timer
from utils.field_constants import *
import robot_constants

from photonlibpy import EstimatedRobotPose

from wpimath.geometry import Translation2d

class FieldOdometry:
    def __init__(self, drivetrain: CommandSwerveDrivetrain, cams: list[PhotonCamCustom] ):
        self.drivetrain = drivetrain
        self.cams = cams
        self.last_update = 0.0
        self.use_vision = True
        self.cam_last_update_times = list()
        for cam in self.cams:
            self.cam_last_update_times.append((cam, self.last_update))

    def enable(self):
        self.use_vision = True

    def disable(self):
        self.use_vision = False

    def get_alliance_hub_tags(self) -> list[int]:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return [2, 3, 4, 5, 8, 9, 10, 11]
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return [18, 19, 20, 21, 24, 25, 26, 27]

    def add_vision_measure(self, cam: PhotonCamCustom, estimated_pose: EstimatedRobotPose):
        self.cam_name = cam.name
        self.pose = estimated_pose.estimatedPose.toPose2d()
        self.time = estimated_pose.timestampSeconds

        tags = estimated_pose.targetsUsed
        tag_count = len(tags)

        std_dev = 2
        std_dev_rot = 100

        if tag_count == 0:
            return
        
        ids = [tag.fiducialId for tag in tags]
        primary_id = ids[0]
        distance_to_target = tags[0].getBestCameraToTarget().translation().toTranslation2d().distance(Translation2d(0, 0))

        # total_tag_area = sum([tag.getArea() for tag in tags])

        if tag_count == 1:
            if distance_to_target > robot_constants.odometry_tag_distance or tags[0].poseAmbiguity > 0.2:
                return
            std_dev = 1

        elif tag_count == 2:
            std_dev = 0.7
            if self.cam_name == robot_constants.front_cam_name:
                std_dev = 0.2 if primary_id in self.get_alliance_hub_tags() else 0.3
            else:
                if distance_to_target <= 0.75:
                    std_dev = 0.4

        else:  # tag_count >= 3
            std_dev = 0.2
            if primary_id in self.get_alliance_hub_tags():
                std_dev = 0.1
                std_dev_rot = 5

        self.drivetrain.add_vision_measurement(self.pose, self.time, (std_dev, std_dev, std_dev_rot))

    def update(self):
        if not self.use_vision:
            return
        
        now = Timer.getFPGATimestamp()
        if now - self.last_update < 0.01: # 10 hz
            return
        self.last_update = now
        self.loop_counter += 1

        for i, (cam, last_update) in enumerate(self.cam_last_update_times):
            if cam.name == robot_constants.front_cam_name and self.loop_counter % 3 == 0:
                ests = cam.get_unread_results()
                if ests:
                    for i in ests:
                        self.add_vision_measure(cam, ests[i]) 
                        self.cam_last_update_times[i] = (cam, now)
            else:
                if last_update == now:
                    est = cam.get_results()
                    if est:
                        self.add_vision_measure(cam, est)
                        self.cam_last_update_times[i] = (cam, now)
                


        
