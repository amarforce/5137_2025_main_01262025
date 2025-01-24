package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ArmConstants {
    // Motor ID
    public static final int motorId = 20;

    // Encoder transform
    public static final double armOffset = 0.0;
    public static final double gearRatio = 100.0;
    
    // PID constants
    public static final double kP = 30;
    public static final double kI = 0;
    public static final double kD = 1;

    // Feedforward constants
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;

    // Arm tolerance
    public static final double armTolerance = 0.1;

    // Arm goals
    public static final double goal1 = Units.degreesToRadians(0); // enter low goal
    public static final double goal2 = Units.degreesToRadians(25); // enter level 2 goal
    public static final double goal3 = Units.degreesToRadians(50); // enter level 3 goal
    public static final double goal4 = Units.degreesToRadians(75); // enter high goal
    public static final double defaultGoal = Units.degreesToRadians(90); // enter high goal
    
    // Simulation constants
    public static final double minAngle = Units.degreesToRadians(-43);
    public static final double maxAngle = Units.degreesToRadians(180);
    public static final double jkg = 1.2;
    public static final double armLength = 0.594;
    public static final DCMotor motorSim = DCMotor.getKrakenX60(1);
    public static final double simPeriod = 0.02;

    // Mechanism constants
    public static final int mechWidth = 20;
    public static final int mechHeight = 50;
}
