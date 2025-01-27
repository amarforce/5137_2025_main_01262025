package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MultiCommands;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.logging.CoralLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class L4PlacementMonitor extends Command {
    private final MultiCommands multiCommands;
    private final CoralLogger logger = new CoralLogger();
    
    public L4PlacementMonitor(MultiCommands multiCommands) {
        this.multiCommands = multiCommands;
        // Add subsystem requirements
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

        // Update dashboard with current positions
        SmartDashboard.putNumber("L4 Placement/Elevator Height", 
            multiCommands.getElevator().getMeasurement());
        SmartDashboard.putNumber("L4 Placement/Arm Angle", 
            multiCommands.getArm().getMeasurement());
        SmartDashboard.putNumber("L4 Placement/Wrist Angle", 
            multiCommands.getWrist().getMeasurement());
        
        // Status indicators
        SmartDashboard.putBoolean("L4 Placement/At Target Height", 
            isAtTargetHeight());
        SmartDashboard.putBoolean("L4 Placement/Systems Ready", 
            areSystemsReady());

        // Log positions
        logger.logPositioning(elevatorHeight, armAngle, wristAngle);
        
        // Update dashboard
        SmartDashboard.putNumber("L4 Placement/Elevator Height", elevatorHeight);
        SmartDashboard.putNumber("L4 Placement/Arm Angle", armAngle);
        SmartDashboard.putNumber("L4 Placement/Wrist Angle", wristAngle);
        
        // Log system status
        boolean atTarget = isAtTargetHeight();
        boolean systemsReady = areSystemsReady();
        logger.logData("L4_MONITOR", "STATUS", 
            String.format("At Target: %b, Systems Ready: %b", 
                atTarget, systemsReady));
        
    }
    
    private boolean isAtTargetHeight() {
        return Math.abs(multiCommands.getElevator().getMeasurement() - 
            VisionConstants.l4_height) < ElevatorConstants.elevatorTol;
    }
    
    private boolean areSystemsReady() {
        return multiCommands.getElevator().atSetpoint() &&
               multiCommands.getArm().atSetpoint() &&
               multiCommands.getWrist().atSetpoint();
    }
}