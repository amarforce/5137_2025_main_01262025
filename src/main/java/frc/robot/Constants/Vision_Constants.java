package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Vision_Constants {
    public static final Transform3d robotToFrontCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d robotToLeftCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
}