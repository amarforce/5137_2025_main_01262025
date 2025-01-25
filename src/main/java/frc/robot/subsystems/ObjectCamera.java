package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.other.DetectedObject;

public class ObjectCamera extends SubsystemBase{
    private PhotonCamera cam;
    private Transform3d robotToCamera;
    private List<PhotonTrackedTarget> newTargets;

    public ObjectCamera(String name,Transform3d robotToCamera){
        this.robotToCamera=robotToCamera;
        cam=new PhotonCamera(name);
        newTargets=new ArrayList<>();
    }

    public List<DetectedObject> getNewObjects(Pose2d robotPose){
        var detectedObjects=new ArrayList<DetectedObject>();
        for(PhotonTrackedTarget target:newTargets){
            Pose3d robotPose3d=new Pose3d(robotPose);
            Pose3d targetPose=robotPose3d.transformBy(robotToCamera).transformBy(target.getBestCameraToTarget());
            Translation3d targetTrans=targetPose.getTranslation();
            int classId=target.objDetectId;
            detectedObjects.add(new DetectedObject(targetTrans, classId));
        }
        return detectedObjects;
    }

    @Override
    public void periodic(){
        var results = cam.getAllUnreadResults();
        for(PhotonPipelineResult res:results){
            newTargets.addAll(res.getTargets());
        }
    }
}
