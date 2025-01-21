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

    public static final Pose2d reefA = new Pose2d(new Translation2d(3.15, 4.19), new Rotation2d(0.0));
    public static final Pose2d reefB = new Pose2d(new Translation2d(3.15, 3.86), new Rotation2d(0.0));
    public static final Pose2d reefC = new Pose2d(new Translation2d(3.685, 2.98), new Rotation2d(Math.PI/3));
    public static final Pose2d reefD = new Pose2d(new Translation2d(3.965, 2.82), new Rotation2d(Math.PI/3));
    public static final Pose2d reefE = new Pose2d(new Translation2d(4.985, 2.82), new Rotation2d(2*Math.PI/3));
    public static final Pose2d reefF = new Pose2d(new Translation2d(5.265, 2.98), new Rotation2d(2*Math.PI/3));
    public static final Pose2d reefG = new Pose2d(new Translation2d(5.8, 3.86), new Rotation2d(Math.PI));
    public static final Pose2d reefH = new Pose2d(new Translation2d(5.8, 4.19), new Rotation2d(Math.PI));
    public static final Pose2d reefI = new Pose2d(new Translation2d(5.265, 5.07), new Rotation2d(4*Math.PI/3));
    public static final Pose2d reefJ = new Pose2d(new Translation2d(4.985, 5.23), new Rotation2d(4*Math.PI/3));
    public static final Pose2d reefK = new Pose2d(new Translation2d(3.965, 5.23), new Rotation2d(5*Math.PI/3));
    public static final Pose2d reefL = new Pose2d(new Translation2d(3.685, 5.07), new Rotation2d(5*Math.PI/3));
    public static final Pose2d reefAB = new Pose2d(new Translation2d(3.15, 4.025), new Rotation2d(0.0));
    public static final Pose2d reefCD = new Pose2d(new Translation2d(3.825, 2.9), new Rotation2d(Math.PI/3));
    public static final Pose2d reefEF = new Pose2d(new Translation2d(5.125, 2.9), new Rotation2d(2*Math.PI/3));
    public static final Pose2d reefGH = new Pose2d(new Translation2d(5.8, 4.025), new Rotation2d(Math.PI));
    public static final Pose2d reefIJ = new Pose2d(new Translation2d(5.125, 5.15), new Rotation2d(4*Math.PI/3));
    public static final Pose2d reefKL = new Pose2d(new Translation2d(3.825, 5.15), new Rotation2d(5*Math.PI/3));

    public static final Pose2d[] leftReef = {reefA, reefC, reefE, reefG, reefI, reefK};
    public static final Pose2d[] centerReef = {reefAB, reefCD, reefEF, reefGH, reefIJ, reefKL};
    public static final Pose2d[] rightReef = {reefB, reefD, reefF, reefH, reefJ, reefL};

    public static final double fieldLength = 17.55;
    public static final double fieldWidth = 8.05;
}
