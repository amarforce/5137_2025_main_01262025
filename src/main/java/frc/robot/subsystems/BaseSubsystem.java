package frc.robot.subsystems;

import frc.robot.utils.logging.RobotLogger;

/**
 * Base class for all subsystems providing common functionality.
 * Extends {@link edu.wpi.first.wpilibj2.command.SubsystemBase} and adds logging and state management.
 */
public abstract class BaseSubsystem extends edu.wpi.first.wpilibj2.command.SubsystemBase {
    protected final RobotLogger logger;
    protected boolean isEnabled = false;
    protected String lastError = "";
    
    /**
     * Constructs a BaseSubsystem with a default logger instance.
     */
    public BaseSubsystem() {
        logger = RobotLogger.getInstance();
    }
    
    /**
     * Check if subsystem is ready for operation.
     * This method should be implemented by subclasses to define subsystem-specific readiness checks.
     * @return true if the subsystem is ready, false otherwise.
     */
    public abstract boolean isReady();
    
    /**
     * Check if subsystem is at its target setpoint.
     * This method should be implemented by subclasses to define what it means for the subsystem to be at its setpoint.
     * @return true if the subsystem is at its setpoint, false otherwise.
     */
    public abstract boolean atSetpoint();
    
    /**
     * Get current measurement from the subsystem.
     * This method should be implemented by subclasses to return the primary measurement of the subsystem.
     * @return The current measurement of the subsystem.
     */
    public abstract double getMeasurement();
} 