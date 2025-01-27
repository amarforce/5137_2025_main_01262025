// src/main/java/frc/robot/commands/monitoring/monitors/L4PlacementMonitor.java
package frc.robot.commands.monitoring.monitors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MultiCommands;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.PlacementConstants;
import frc.robot.utils.logging.CoralLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class L4PlacementMonitor extends Command {
    private final MultiCommands multiCommands;
    private final CoralLogger logger = new CoralLogger();
    
    public L4PlacementMonitor(MultiCommands multiCommands) {
        this.multiCommands = multiCommands;
        addRequirements(multiCommands.getElevator(), 
                       multiCommands.getArm(), 
                       multiCommands.getWrist());
    }
    
    /**
     * This method is called periodically (about every 20ms) while the command is scheduled.
     * It reads the current measurements from the elevator, arm, and wrist subsystems,
     * logs these positions, updates the SmartDashboard with these values,
     * checks if the elevator is at the target height for L4 placement,
     * checks if all systems (elevator, arm, wrist) are ready (at setpoint),
     * updates the SmartDashboard with the status indicators,
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
        SmartDashboard.putNumber("L4 Placement/Elevator Height", elevatorHeight);
        SmartDashboard.putNumber("L4 Placement/Arm Angle", armAngle);
        SmartDashboard.putNumber("L4 Placement/Wrist Angle", wristAngle);
        
        // Status indicators
        boolean atTarget = isAtTargetHeight();
        boolean systemsReady = areSystemsReady();
        
        SmartDashboard.putBoolean("L4 Placement/At Target Height", atTarget);
        SmartDashboard.putBoolean("L4 Placement/Systems Ready", systemsReady);
        
        // Log system status
        logger.logData("STATUS", 
            String.format("At Target: %b, Systems Ready: %b", 
                atTarget, systemsReady));
    }
    
    /**
     * Checks if the elevator is at the target height for L4 placement.
     * It calculates the absolute difference between the current elevator height and the target L4 height,
     * and returns true if this difference is within the allowed tolerance defined in ElevatorConstants.
     *
     * @return true if the elevator is at the target height, false otherwise.
     */
    private boolean isAtTargetHeight() {
        return Math.abs(multiCommands.getElevator().getMeasurement() - 
            PlacementConstants.L4_HEIGHT) < ElevatorConstants.elevatorTol;
    }
    
    /**
     * Checks if all necessary systems (elevator, arm, wrist) are ready for L4 placement.
     * Systems are considered ready if they are at their respective setpoints.
     *
     * @return true if all systems are ready, false otherwise.
     */
    private boolean areSystemsReady() {
        return multiCommands.getElevator().atSetpoint() &&
               multiCommands.getArm().atSetpoint() &&
               multiCommands.getWrist().atSetpoint();
    }
}