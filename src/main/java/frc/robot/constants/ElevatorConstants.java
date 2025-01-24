package frc.robot.constants;

public class ElevatorConstants {
    // Motor IDs
    public static final int leftMotorId = 21; // TODO: Change to 1
    public static final int rightMotorId = 22; // TODO: Change to 2

    // Encoder transform
    public static final double elevatorOffset = 0;
    public static final double metersPerRotation = 0.01397;

    // Feedforward constants
    public static final double kS = 0.1;
    public static final double kG = 0.1;
    public static final double kV = 0.1;

    // PID constants
    public static final double kP = 30;
    public static final double kI = 0;
    public static final double kD = 1;

    // Elevator tolerance
    public static final double elevatorTol = 0.1;

    // Elevator goals
    public static final double goal1 = 0.06;
    public static final double goal2 = 0.26;
    public static final double goal3 = 0.56;
    public static final double goal4 = 1.26;
    public static final double sourceGoal = 0.76;
    public static final double groundIntakeGoal = 0.26;
    public static final double defaultGoal = 0;

    // Simulation constants
    public static final double elevatorGearing = 20.0; // gear ratio
    public static final double carriageMass = 13.0; // in kg
    public static final double drumRadius = 0.0445; // in meters
    public static final double minHeight = 0; // in meters
    public static final double maxHeight = 1.27; // in meters
    public static final double startingHeight = minHeight; // in meters
    public static final double simPeriod = 0.02;

    // Mech constants
    public static final double mechWidth = 20;
    public static final double mechHeight = 50;
}
