package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision_Constants;

public class Vision extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout;

    private PhotonCamera frontCamera;
    private PhotonPoseEstimator frontPoseEstimator;

    public Vision() {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        frontCamera = new PhotonCamera("frontCamera");
        frontPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision_Constants.robotToFrontCamera);
    }

    public Optional<EstimatedRobotPose> getFrontPose() {
        return frontPoseEstimator.update(frontCamera.getLatestResult());
    }
}