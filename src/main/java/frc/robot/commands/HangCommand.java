package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang;
import frc.robot.utils.logging.RobotLogger;

public class HangCommand extends Command {
    private final Hang hang;
    private final RobotLogger logger;
    private boolean isFinished = false;
    
    public HangCommand(Hang hang) {
        this.hang = hang;
        this.logger = RobotLogger.getInstance();
        addRequirements(hang);
    }
    
    @Override
    public void initialize() {
        logger.logData("HangCommand", "Status", "Command initialized");
        isFinished = false;
        
        // Check if system is ready
        if (!hang.isReady()) {
            logger.logData("HangCommand", "Error", "System not ready - insufficient pressure");
            isFinished = true;
            return;
        }
    }
    
    @Override
    public void execute() {
        // Implement the hanging sequence
        try {
            if (!hang.isClampActivated()) {
                hang.clampActivate();
                logger.logData("HangCommand", "Action", "Activating clamp");
            } else if (!hang.isClimbExtended()) {
                hang.climbExtend();
                logger.logData("HangCommand", "Action", "Extending climb mechanism");
            } else {
                isFinished = true;
                logger.logData("HangCommand", "Status", "Hanging sequence completed");
            }
        } catch (Exception e) {
            logger.logData("HangCommand", "Error", "Error during execution: " + e.getMessage());
            isFinished = true;
        }
    }
    
    @Override
    public boolean isFinished() {
        return isFinished;
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            logger.logData("HangCommand", "Status", "Command interrupted");
        }
    }
}

