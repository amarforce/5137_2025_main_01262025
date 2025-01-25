package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static final Transform3d robotToFrontCamera = new Transform3d(new Translation3d(0.35, 0, 0.2), new Rotation3d(0, -Math.PI/6, 0));
    public static final Transform3d robotToLeftCamera = new Transform3d(new Translation3d(0, 0.35, 0.2), new Rotation3d(0, -Math.PI/6, Math.PI/2));
    public static final Transform3d robotToFrontObjectCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    public static final String[] classNames = {"Algae","Coral"};

    public static final double objectMarginOfError = 0.1;

    // public static final Translation2d reefA = new Translation2d(3.75, 4.19);
    // public static final Translation2d reefB = new Translation2d(3.75, 3.86);
    // public static final Translation2d reefC = new Translation2d(4.00, 3.50);
    // public static final Translation2d reefD = new Translation2d(4.25, 3.30);
    // public static final Translation2d reefE = new Translation2d(4.70, 3.30);
    // public static final Translation2d reefF = new Translation2d(4.95, 3.50);
    // public static final Translation2d reefG = new Translation2d(5.25, 3.86);
    // public static final Translation2d reefH = new Translation2d(5.25, 4.19);
    // public static final Translation2d reefI = new Translation2d(4.95, 4.60);
    // public static final Translation2d reefJ = new Translation2d(4.70, 4.80);
    // public static final Translation2d reefK = new Translation2d(4.25, 4.80);
    // public static final Translation2d reefL = new Translation2d(4.00, 4.60);
    public static final Translation3d[][] coralPositions = new Translation3d[GeneralConstants.sides*2][3];

    public static final Translation2d reefCenter = new Translation2d(4.4958,4.0259);
    public static final double d1 = 0.75;
    public static final double d2 = 0.16;
    public static final double l2_d1 = 0.1;
    public static final double l2_height = 0.740;
    public static final double l3_d1 = 0.1;
    public static final double l3_height = 1.143;
    public static final double l4_d1 = 0.2;
    public static final double l4_height = 1.6;
    static{
        for(int i=0;i<GeneralConstants.sides*2;i++){
            Rotation2d angle=new Rotation2d(2*(i/2)*Math.PI/GeneralConstants.sides).rotateBy(Rotation2d.k180deg);
            Translation2d center=reefCenter.plus(new Translation2d(d1, angle));
            Translation2d side=center.plus(new Translation2d(d2, angle.rotateBy(i%2==0?Rotation2d.kCW_90deg:Rotation2d.kCCW_90deg)));
            coralPositions[i][0]=new Translation3d(side.plus(new Translation2d(l2_d1, angle))).plus(new Translation3d(0, 0, l2_height));
            coralPositions[i][1]=new Translation3d(side.plus(new Translation2d(l3_d1, angle))).plus(new Translation3d(0, 0, l3_height));
            coralPositions[i][2]=new Translation3d(side.plus(new Translation2d(l4_d1, angle))).plus(new Translation3d(0, 0, l4_height));
        }
    }
}
