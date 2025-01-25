package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.opencv.aruco.EstimateParameters;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagCamera extends SubsystemBase{
    private PhotonCamera cam;
    private PhotonPoseEstimator estimator;
    private Transform3d robotToCamera;
    private List<EstimatedRobotPose> estimatedPoses;

    public AprilTagCamera(String name,Transform3d robotToCamera,AprilTagFieldLayout fieldLayout){
        this.robotToCamera=robotToCamera;
        cam=new PhotonCamera(name);
        estimator=new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        estimatedPoses=new ArrayList<>();
    }

    public void startSim(VisionSystemSim sim){
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        cameraProp.setFPS(50);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        PhotonCameraSim cameraSim = new PhotonCameraSim(cam, cameraProp);
        sim.addCamera(cameraSim, robotToCamera);
        if(Robot.isSimulation()){
            cameraSim.enableDrawWireframe(true);
        }
    }

    public List<EstimatedRobotPose> getNewPoses(){
        var res=new ArrayList<>(estimatedPoses);
        estimatedPoses.clear();
        return res;
    }

    @Override
    public void periodic(){
        var results = cam.getAllUnreadResults();
        for(PhotonPipelineResult res:results){
            var estimatedPose=estimator.update(res);
            if(estimatedPose.isPresent()){
                estimatedPoses.add(estimatedPose.get());
            }
        }
    }
}
