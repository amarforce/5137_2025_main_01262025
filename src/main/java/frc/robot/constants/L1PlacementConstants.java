// src/main/java/frc/robot/constants/L1PlacementConstants.java
package frc.robot.constants;

public final class L1PlacementConstants {
    // Movement timeouts - in seconds, maximum time allowed for each stage of L1 placement
    public static final double APPROACH_TIMEOUT = 3.0;
    // Maximum time allowed for final positioning adjustments before placement
    public static final double POSITIONING_TIMEOUT = 1.0;
    // Maximum time allowed for retracting mechanisms after placement
    public static final double RETRACTION_TIMEOUT = 2.0;
    
    // Timing constants - in seconds, durations for specific actions during placement
    public static final double STABILITY_WAIT_TIME = 0.1; // Time to wait for mechanisms to stabilize before release
    public static final double RELEASE_TIME = 0.3; // Duration for releasing the game piece
    
    // Position tolerances (might be different for L1) - acceptable deviation from target positions
    public static final double HEIGHT_TOLERANCE = ElevatorConstants.elevatorTol; // Tolerance for elevator height, using elevator's default tolerance
    public static final double ANGLE_TOLERANCE = ArmConstants.armTolerance; // Tolerance for arm angle, using arm's default tolerance
    
    // Safety margins - values to ensure safe operation and prevent collisions
    public static final double MIN_HEIGHT_MARGIN = 0.05; // meters, minimum height margin above obstacles during movement
    public static final double GROUND_CLEARANCE = 0.02; // meters, distance to maintain above the ground
}