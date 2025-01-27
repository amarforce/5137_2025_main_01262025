    /**
     * Constants for L4 placement
     */
    private static final class L4PlacementConstants {
        // Height constants
        public static final double TARGET_HEIGHT = VisionConstants.l4_height; // 1.6m
        public static final double APPROACH_HEIGHT = VisionConstants.l3_height; // 1.143m
        
        // Timing constants
        public static final double STABILITY_WAIT_TIME = 0.2; // seconds
        public static final double RELEASE_TIME = 0.5; // seconds
        
        // Position tolerances
        public static final double HEIGHT_TOLERANCE = ElevatorConstants.elevatorTol;
        public static final double ANGLE_TOLERANCE = ArmConstants.armTolerance;

        public static final double MOVEMENT_TIMEOUT = 2.0;
        public static final double STABILITY_WAIT = 0.2;
        public static final double RELEASE_TIME = 0.5;
        public static final double SAFETY_MARGIN = 0.05;
    }