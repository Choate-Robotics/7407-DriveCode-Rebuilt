import ntcore

from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.geometry import Transform3d, Pose3d, Translation2d, Pose2d
from wpilib import TimedRobot


class PhotonCamCustom:
    def __init__(self, name: str, robot_to_camera: Transform3d):
        self.cam = PhotonCamera(name)
        self.name = name
        self.robot_to_camera = robot_to_camera
        self.estimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark),
            self.robot_to_camera
        )
        self.table = (
            ntcore.NetworkTableInstance.getDefault()
            .getTable("Cameras")
            .getSubTable(self.name)
        )
        self.pose_publisher = self.table.getStructTopic("Estimated Pose", Pose2d).publish()
        self.has_target_publisher = self.table.getBooleanTopic("Has target").publish()
        self.targets_publisher = self.table.getIntegerArrayTopic("Targets").publish()
        self.distance_publisher = self.table.getFloatTopic("Distance to target").publish()

    def update_tables(self):
        if not TimedRobot.isSimulation():
            result = self.get_result()

            if result:

                if result.estimatedPose:
                    self.pose_publisher.set(result.estimatedPose.toPose2d())

                has_targets = len(result.targetsUsed) > 0

                self.has_target_publisher.set(has_targets)

                if has_targets:
                    self.targets_publisher.set([target.getFiducialId() for target in result.targetsUsed])
                    self.distance_publisher.set(
                        result.targetsUsed[0].bestCameraToTarget
                        .translation()
                        .toTranslation2d()
                        .distance(Translation2d(0, 0)),
                    )

    def get_estimated_robot_pose(self) -> Pose3d:
        """
        Returns a Pose3d of the estimated robot position
        """
        result = self.cam.getLatestResult()
        est_pose = self.estimator.estimateCoprocMultiTagPose(result)
        if est_pose is None:
            est_pose = self.estimator.estimateLowestAmbiguityPose(result)

        if est_pose is not None:
            return est_pose.estimatedPose
        
        return Pose3d()

    def get_result(self) -> EstimatedRobotPose | None:
        """
        Returns an EstimatedRobotPose, which includes pose, timestamp, tags
        """
        result = self.cam.getLatestResult()
        if result:
            est_pose = self.estimator.estimateCoprocMultiTagPose(result)
            if est_pose is None:
                est_pose = self.estimator.estimateLowestAmbiguityPose(result)

            return est_pose
        return None
    
    def get_unread_results(self) -> list[EstimatedRobotPose] | None:
        """
        Returns a list of EstimatedRobotPose from unread results
        """
        unread_results = self.cam.getAllUnreadResults()
        if unread_results:
            poses = list()
            for result in unread_results:
                est_pose = self.estimator.estimateCoprocMultiTagPose(result)
                if est_pose is None:
                    est_pose = self.estimator.estimateLowestAmbiguityPose(result)
                if est_pose is None:
                    poses.append(est_pose)

            return poses
        return None

