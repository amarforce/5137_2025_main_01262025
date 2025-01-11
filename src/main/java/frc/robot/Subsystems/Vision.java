package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera frontCamera;

    public Vision() {
        frontCamera = new PhotonCamera("frontCamera");
    }

    @Override
    public void periodic() {
        var result = frontCamera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            int id = target.getFiducialId();
            Transform3d pose = target.getBestCameraToTarget();
            System.out.println("Target: " + id + "Pose: " + pose);
        }
    }
}