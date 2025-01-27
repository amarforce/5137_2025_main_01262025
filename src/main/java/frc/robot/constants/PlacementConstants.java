// src/main/java/frc/robot/constants/PlacementConstants.java
package frc.robot.constants;

public final class PlacementConstants {
    // Height constants for different levels (in meters)
    public static final double L1_HEIGHT = 0.2;  // Level 1 height - height of the lowest level
    public static final double L2_HEIGHT = 0.6;  // Level 2 height - height of the second level
    public static final double L3_HEIGHT = 1.0;  // Level 3 height - height of the third level
    public static final double L4_HEIGHT = 1.6;  // Level 4 height - height of the highest level, level 4
    
    // Timing constants
    public static final double STABILITY_WAIT_TIME = 0.2;  // seconds - standard wait time to ensure mechanism stability before action
    public static final double L1_STABILITY_WAIT_TIME = 0.1;  // seconds - reduced stability wait time for level 1, assuming less critical placement
    
    // Release durations
    public static final double STANDARD_RELEASE_TIME = 0.5;  // seconds - standard duration for releasing a game piece
    public static final double L1_RELEASE_TIME = 0.3;  // seconds - shorter release time for level 1, optimized for speed
    
    // Movement timeouts
    public static final double APPROACH_TIMEOUT = 3.0;  // seconds - maximum time allowed for approaching a placement target
    public static final double POSITIONING_TIMEOUT = 1.0;  // seconds - maximum time for fine-tuning position at the target
    public static final double RETRACTION_TIMEOUT = 2.0;  // seconds - maximum time allowed for retracting the mechanism after placement
    
    // Safety margins
    public static final double MIN_HEIGHT_MARGIN = 0.05;  // meters - minimum height margin to avoid ground collisions during movement
    public static final double GROUND_CLEARANCE = 0.02;  // meters -  minimum distance from the ground for safe operation
}