package frc.robot.other;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.VisionConstants;

public class DetectedObject {
    private Translation3d pos;
    private int classId;

    public DetectedObject(Translation3d pos,int classId){
        this.pos=pos;
        this.classId=classId;
    }

    public Translation3d getPos(){
        return pos;
    }

    public int getClassId(){
        return classId;
    }

    public String getClassName(){
        return VisionConstants.classNames[classId];
    }
}
