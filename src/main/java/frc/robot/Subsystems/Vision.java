package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout;

    private PhotonCamera frontCamera;
    private PhotonPoseEstimator frontPoseEstimator;

    private PhotonCamera leftCamera;
    private PhotonPoseEstimator leftPoseEstimator;

    private PhotonCamera frontObjectCamera;

    private VisionSystemSim visionSim;

    private SimCameraProperties frontCameraProp;
    private PhotonCameraSim frontCameraSim;

    private SimCameraProperties leftCameraProp;
    private PhotonCameraSim leftCameraSim;

    public Vision() {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        frontCamera = new PhotonCamera("frontCamera");
        frontPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotToFrontCamera);

        leftCamera = new PhotonCamera("leftCamera");
        leftPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotToLeftCamera);

        frontObjectCamera = new PhotonCamera("frontObjectCamera");

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        frontCameraProp = new SimCameraProperties();
        frontCameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        frontCameraProp.setFPS(50);
        frontCameraProp.setAvgLatencyMs(35);
        frontCameraProp.setLatencyStdDevMs(5);
        frontCameraSim = new PhotonCameraSim(frontCamera, frontCameraProp);
        visionSim.addCamera(frontCameraSim, VisionConstants.robotToFrontCamera);

        leftCameraProp = new SimCameraProperties();
        leftCameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        leftCameraProp.setFPS(50);
        leftCameraProp.setAvgLatencyMs(35);
        leftCameraProp.setLatencyStdDevMs(5);
        leftCameraSim = new PhotonCameraSim(leftCamera, leftCameraProp);
        visionSim.addCamera(leftCameraSim, VisionConstants.robotToLeftCamera);

        if (Robot.isSimulation()) {
            frontCameraSim.enableDrawWireframe(true);
            leftCameraSim.enableDrawWireframe(true);
        }
    }

    //TODO: Switch to new method when docs update

    public Optional<EstimatedRobotPose> getFrontPoseEstimate() {
        return frontPoseEstimator.update(frontCamera.getLatestResult());
    }

    public Optional<EstimatedRobotPose> getLeftPoseEstimate() {
        return leftPoseEstimator.update(leftCamera.getLatestResult());
    }

    public PhotonTrackedTarget getClosestCoral() {
        return frontObjectCamera.getLatestResult().getBestTarget();
    }

    public void updateSim(Pose2d pose) {
        visionSim.update(pose);
    }
}