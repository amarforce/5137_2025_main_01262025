package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class L4PlacementMonitor extends Command {
    private final MultiCommands multiCommands;
    
    public L4PlacementMonitor(MultiCommands multiCommands) {
        this.multiCommands = multiCommands;
        // Add subsystem requirements
        addRequirements(multiCommands.getElevator(), 
                       multiCommands.getArm(), 
                       multiCommands.getWrist());
    }
    
    @Override
    public void execute() {
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