package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout;

    private PhotonCamera frontCamera;
    private PhotonPoseEstimator frontPoseEstimator;

    private PhotonCamera leftCamera;
    private PhotonPoseEstimator leftPoseEstimator;

    private PhotonCamera frontObjectCamera;

    public Vision() {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        frontCamera = new PhotonCamera("frontCamera");
        frontPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotToFrontCamera);

        leftCamera = new PhotonCamera("leftCamera");
        leftPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotToLeftCamera);

        frontObjectCamera = new PhotonCamera("frontObjectCamera");
    }

    //TODO: Switch to new method when docs update

    public Optional<EstimatedRobotPose> getFrontPose() {
        return frontPoseEstimator.update(frontCamera.getLatestResult());
    }

    public Optional<EstimatedRobotPose> getLeftPose() {
        return leftPoseEstimator.update(leftCamera.getLatestResult());
    }

    public PhotonTrackedTarget getClosestCoral() {
        return frontObjectCamera.getLatestResult().getBestTarget();
    }
}
