package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.elastic.Reef;
import frc.robot.other.DetectedObject;

public class Vision extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout;

    private AprilTagCamera[] aprilTagCameras;
    private ObjectCamera[] objectCameras;

    private VisionSystemSim visionSim;

    private Reef reef;

    public Vision(Reef reef) {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        AprilTagCamera frontCamera = new AprilTagCamera("frontCamera", VisionConstants.robotToFrontCamera, fieldLayout);
        AprilTagCamera leftCamera = new AprilTagCamera("leftCamera", VisionConstants.robotToLeftCamera, fieldLayout);
        aprilTagCameras = new AprilTagCamera[]{frontCamera, leftCamera};

        ObjectCamera frontObjectCamera = new ObjectCamera("frontObjectCamera",VisionConstants.robotToFrontObjectCamera);
        objectCameras = new ObjectCamera[]{frontObjectCamera};

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        for(AprilTagCamera cam:aprilTagCameras){
            cam.startSim(visionSim);
        }

        this.reef = reef;
    }

    public List<EstimatedRobotPose> getNewPoses(){
        var res=new ArrayList<EstimatedRobotPose>();
        for(AprilTagCamera cam:aprilTagCameras){
            res.addAll(cam.getNewPoses());
        }
        return res;
    }

    private List<DetectedObject> getNewObjects(Pose2d robotPose) {
        var res=new ArrayList<DetectedObject>();
        for(ObjectCamera cam:objectCameras){
            res.addAll(cam.getNewObjects(robotPose));
        }
        return res;
    }

    public void processNewObjects(Pose2d robotPose){
        List<DetectedObject> objects=getNewObjects(robotPose);
        for(DetectedObject object:objects){
            if(object.getClassName()!="Coral"){
                continue;
            }
            Translation3d coralPos=object.getPos();

        }
    }

    public int checkObjectOnReef(Translation3d target3d) {
        int closest = 0;
        double closest_distance = 100.0;
        Translation2d[] reefPositions = VisionConstants.reefPositions;
        for (int i = 0; i < reefPositions.length; i++) {
            Translation2d diff = target3d.toTranslation2d().minus(reefPositions[i]);
            double distance = Math.hypot(diff.getX(), diff.getY());
            if (distance < closest_distance) {
                closest_distance = distance;
                closest = i;
            }
        }
        Translation2d closestReef = reefPositions[closest];
        Translation2d diff = target3d.toTranslation2d().minus(closestReef);
        if (Math.abs(diff.getX()) <= VisionConstants.objectMarginOfError && Math.abs(diff.getY()) <= VisionConstants.objectMarginOfError) {
            double height = target3d.getZ();
            if (Math.abs(height - VisionConstants.L4) < VisionConstants.objectMarginOfError) {
                return 4 + closest*5;
            } else if (Math.abs(height - VisionConstants.L3) < VisionConstants.objectMarginOfError) {
                return 3 + closest*5;
            } else if (Math.abs(height - VisionConstants.L2) < VisionConstants.objectMarginOfError) {
                return 2 + closest*5;
            } else if (Math.abs(height - VisionConstants.L1) < VisionConstants.objectMarginOfError) {
                return 1 + closest*5;
            } else {
                return 0 + closest*5;
            }
        } else {
            return 0;
        }
    };

    public void updateSim(Pose2d currentPose) {
        visionSim.update(currentPose);
    }
}
