package frc.robot.other;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.constants.GeneralConstants;

public class RobotUtils {
    public static boolean onRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public static Pose2d invertPose(Pose2d pose) {
        return new Pose2d(GeneralConstants.fieldLength - pose.getX(), GeneralConstants.fieldWidth - pose.getY(), pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    public static Pose2d invertPoseToAlliance(Pose2d pose){
        if(onRedAlliance()){
            return invertPose(pose);
        }else{
            return pose;
        }
    }

    public static Translation3d invertTrans3d(Translation3d trans) {
        return new Translation3d(GeneralConstants.fieldLength - trans.getX(), GeneralConstants.fieldWidth - trans.getY(), trans.getZ());
    }

    public static Translation3d invertTrans3dToAlliance(Translation3d trans) {
        if(onRedAlliance()){
            return invertTrans3d(trans);
        }else{
            return trans;
        }
    }

    public static Rotation2d getPerspectiveForward(){
        if(Robot.isSimulation()){
            return Rotation2d.kCCW_90deg;
        }else if(onRedAlliance()){
            return Rotation2d.k180deg;
        }else{
            return Rotation2d.kZero;
        }
    }

    public static Pose2d getClosestPoseToPose(Pose2d pose,Pose2d[] others) {
        pose = RobotUtils.invertPoseToAlliance(pose);
        Pose2d closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (Pose2d other : others) {
            Transform2d transform = pose.minus(other);
            double distance = Math.hypot(transform.getX(), transform.getY());
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = other;
            }
        }
        return closest;
    }
}
