package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {

    public static final double armSpeed = 3.0;
    public static final int motorId  = 20;
    public static final double kP = 30; // enter
    public static final double kI = 0; // enter
    public static final double kD = 1; // enter
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double min = 0; // pi /180
    public static final double max = 100;
    public static final double tolerance = .1;
    public static final double goal1 = 0; //enter low goal
    public static final double goal2 = 0; //enter level 2 goal
    public static final double goal3 = 0; //enter level 3 goal
    public static final double goal4 = 0; // enter high goal
    public static final double armOffset = 0.0;
    public static final double gearRatio = 1.0;
    public static final double jkg = 500.0;
    public static final double armLength = 0.594;
    public static final DCMotor motorSim = DCMotor.getKrakenX60(1);
    public static final double simPeriod = 0.02;
    public static final int mechWidth = 20;
    public static final int mechHeight = 50;
}
