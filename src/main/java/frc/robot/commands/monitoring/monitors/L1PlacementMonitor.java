// src/main/java/frc/robot/commands/monitoring/monitors/L1PlacementMonitor.java
package frc.robot.commands.monitoring.monitors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MultiCommands;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.logging.CoralLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class L1PlacementMonitor extends Command {
    private final MultiCommands multiCommands;
    private final CoralLogger logger = new CoralLogger();
    
    public L1PlacementMonitor(MultiCommands multiCommands) {
        this.multiCommands = multiCommands;
        addRequirements(multiCommands.getElevator(), 
                       multiCommands.getArm(), 
                       multiCommands.getWrist());
    }
    
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
        logger.logData("L1_MONITOR", "STATUS", 
            String.format("At Target: %b, Systems Ready: %b", 
                atTarget, systemsReady));
    }
    
    private boolean isAtTargetHeight() {
        return Math.abs(multiCommands.getElevator().getMeasurement() - 
            VisionConstants.l1_height) < ElevatorConstants.elevatorTol;
    }
    
    private boolean areSystemsReady() {
        return multiCommands.getElevator().atSetpoint() &&
               multiCommands.getArm().atSetpoint() &&
               multiCommands.getWrist().atSetpoint() &&
               !multiCommands.getElevator().isAtLowerLimit();
    }
}