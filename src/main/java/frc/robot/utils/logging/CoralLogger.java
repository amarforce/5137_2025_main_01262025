// src/main/java/frc/robot/utils/logging/CoralLogger.java
package frc.robot.utils.logging;

public class CoralLogger {
    private final RobotLogger logger;
    private static final String SUBSYSTEM = "Coral";
    
    public CoralLogger() {
        logger = RobotLogger.getInstance();
    }
    
    /**
     * Logs the start of a placement action.
     * @param level The level of the placement (e.g., 1, 2, 3).
     */
    public void logPlacementStart(int level) {
        logger.logData(SUBSYSTEM, "PLACEMENT_START", 
            "Level: L" + level);
    }
    
    /**
     * Logs the completion of a placement action.
     * @param level The level of the placement (e.g., 1, 2, 3).
     * @param success Whether the placement was successful.
     */
    public void logPlacementComplete(int level, boolean success) {
        logger.logData(SUBSYSTEM, "PLACEMENT_COMPLETE", 
            String.format("Level: L%d, Success: %b", level, success));
    }
    
    /**
     * Logs the current positioning of the mechanism.
     * @param elevatorHeight The height of the elevator.
     * @param armAngle The angle of the arm.
     * @param wristAngle The angle of the wrist.
     */
    public void logPositioning(double elevatorHeight, double armAngle, double wristAngle) {
        logger.logData(SUBSYSTEM, "POSITION_UPDATE", 
            String.format("Elevator: %.2f, Arm: %.2f, Wrist: %.2f", 
                elevatorHeight, armAngle, wristAngle));
    }
    
    /**
     * Logs an error with a specific type and details.
     * @param errorType The type of error (e.g., "Motor", "Sensor").
     * @param details Detailed description of the error.
     */
    public void logError(String errorType, String details) {
        logger.logData(SUBSYSTEM, "ERROR", 
            errorType + ": " + details);
    }
}