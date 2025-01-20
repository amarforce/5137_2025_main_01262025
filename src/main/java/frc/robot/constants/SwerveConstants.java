package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConstants {
    public static final double translationalDeadband = 0.1;
    public static final double rotationalDeadband = 0.1;
    public static final double odometryFrequency = 20;

    public static final double translation_kP = 5.0;
    public static final double translation_kI = 0.0;
    public static final double translation_kD = 0.0;
    public static final double rotation_kP = 5.0;
    public static final double rotation_kI = 0.0;
    public static final double rotation_kD = 0.0;

    public static final Pose2d reefA = new Pose2d(new Translation2d(3.15, 4.1525), new Rotation2d(0.0));
    public static final Pose2d reefB = new Pose2d(new Translation2d(3.15, 3.8525), new Rotation2d(0.0));
    public static final Pose2d reefC = new Pose2d();
    public static final Pose2d reefD = new Pose2d();
    public static final Pose2d reefE = new Pose2d();
    public static final Pose2d reefF = new Pose2d();
    public static final Pose2d reefG = new Pose2d();
    public static final Pose2d reefH = new Pose2d();
    public static final Pose2d reefI = new Pose2d();
    public static final Pose2d reefJ = new Pose2d();
    public static final Pose2d reefK = new Pose2d();
    public static final Pose2d reefL = new Pose2d();
    public static final Pose2d reefAB = new Pose2d(new Translation2d(3.15, 4.025), new Rotation2d(0.0));
    public static final Pose2d reefCD = new Pose2d();
    public static final Pose2d reefEF = new Pose2d();
    public static final Pose2d reefGH = new Pose2d();
    public static final Pose2d reefIJ = new Pose2d();
    public static final Pose2d reefKL = new Pose2d();

    public static final double fieldLength = 17.55;
    public static final double fieldWidth = 8.05;
}
