package frc.robot.constants;

/**
 * HangConstants.java
 * 
 * This class contains constant values used in the Hang subsystem of the robot.
 * These constants are essential for configuring the solenoids, pressure thresholds,
 * and timing for various operations related to the hanging mechanism.
 * 
 * Constants:
 * - clampSolenoid: The channel number for the solenoid that controls the clamp.
 * - climbSolenoid: The channel number for the solenoid that controls the climbing mechanism.
 * - minPressure: The minimum pressure threshold required to activate the compressor.
 * - maxPressure: The maximum pressure threshold at which the compressor will turn off.
 * - clampDeactivationTime: The time duration for which the clamp remains deactivated.
 * - climbExtensionTime: The time duration for which the climbing mechanism extends.
 * - clampActivationTime: The time duration for which the clamp remains activated.
 * 
 * Logging Constants:
 * - SUBSYSTEM_NAME: The name of the subsystem used for logging purposes.
 * - LOGGING_INTERVAL: The interval at which logging occurs, measured in seconds.
 * 
 * Usage:
 * These constants should be referenced throughout the Hang subsystem to ensure
 * consistent behavior and easy adjustments to parameters without modifying multiple
 * locations in the code.
 */

public class HangConstants {
    public static final int clampSolenoid = 1; // Channel for Solenoid 1
    public static final int climbSolenoid = 2; // Channel for Solenoid 2
    public static final double minPressure = 70; // Min pressure threshold to turn the compressor on
    public static final double maxPressure = 120; // Max pressure threshold to turn the compressor off
    public static final double clampDeactivationTime = 1.0;
    public static final double climbExtensionTime = 1.0;
    public static final double clampActivationTime = 1.0;

    // Logging constants
    public static final String SUBSYSTEM_NAME = "Hang";
    public static final double LOGGING_INTERVAL = 0.1; // seconds
}
