// src/main/java/frc/robot/commands/monitoring/monitors/L1PlacementMonitor.java
package frc.robot.commands.monitoring.monitors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MultiCommands;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.PlacementConstants;
import frc.robot.utils.logging.CoralLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class L1PlacementMonitor extends Command {
    private final MultiCommands multiCommands;
    private final CoralLogger logger = new CoralLogger();
    
    /**
     * Constructor for L1PlacementMonitor.
     * @param multiCommands An instance of MultiCommands containing references to subsystem commands.
     */
    public L1PlacementMonitor(MultiCommands multiCommands) {
        this.multiCommands = multiCommands;
        addRequirements(multiCommands.getElevator(), 
                       multiCommands.getArm(), 
                       multiCommands.getWrist());
    }
    
    /**
     * Called repeatedly when this Command is scheduled to run.
     * Monitors and logs the elevator, arm, and wrist positions,
     * updates the SmartDashboard with current and target statuses,
     * and logs the overall system status.
     */
    @Override
    public void execute() {
        double elevatorHeight = multiCommands.getElevator().getMeasurement();
        double armAngle = multiCommands.getArm().getMeasurement();
        double wristAngle = multiCommands.getWrist().getMeasurement();
        
        // Log positions
        logger.logPositioning(elevatorHeight, armAngle, wristAngle);

        // Update dashboard
        SmartDashboard.putNumber("L1 Placement/Elevator Height", elevatorHeight);
        SmartDashboard.putNumber("L1 Placement/Arm Angle", armAngle);
        SmartDashboard.putNumber("L1 Placement/Wrist Angle", wristAngle);
        
        // Status indicators
        boolean atTarget = isAtTargetHeight();
        boolean systemsReady = areSystemsReady();
        
        SmartDashboard.putBoolean("L1 Placement/At Target Height", atTarget);
        SmartDashboard.putBoolean("L1 Placement/Systems Ready", systemsReady);
        
        // Log system status
        logger.logData("STATUS", 
            String.format("At Target: %b, Systems Ready: %b", 
                atTarget, systemsReady));
    }
    
    /**
     * Checks if the elevator is at the target height for L1 placement.
     * @return true if the elevator height is within the tolerance of the L1 target height, false otherwise.
     */
    private boolean isAtTargetHeight() {
        return Math.abs(multiCommands.getElevator().getMeasurement() - 
            PlacementConstants.L1_HEIGHT) < ElevatorConstants.elevatorTol;
    }
    
    /**
     * Checks if all necessary systems (elevator, arm, wrist) are ready for L1 placement.
     * Systems are considered ready if they are at their setpoints and the elevator is not at its lower limit.
     * @return true if all systems are ready, false otherwise.
     */
    private boolean areSystemsReady() {
        return multiCommands.getElevator().atSetpoint() &&
               multiCommands.getArm().atSetpoint() &&
               multiCommands.getWrist().atSetpoint() &&
               !multiCommands.getElevator().isAtLowerLimit();
    }
}