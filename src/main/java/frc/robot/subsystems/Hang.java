package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.HangConstants;

/**
 * The Hang class represents a subsystem responsible for managing the 
 * hanging mechanism of the robot, including the clamp and climb functionalities.
 * It extends the BaseSubsystem class to inherit common subsystem behaviors.
 */
public class Hang extends BaseSubsystem {
    private final Solenoid clampSolenoid; // Solenoid controlling the clamp mechanism
    private final Solenoid climbSolenoid; // Solenoid controlling the climbing mechanism
    private final Compressor compressor; // Compressor to manage pneumatic pressure for the system
    
    /**
     * Constructor for the Hang class. Initializes the solenoids and compressor,
     * and sets the pressure limits for the compressor.
     */
    public Hang() {
        super(); // Call the constructor of the BaseSubsystem
        clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, HangConstants.clampSolenoid); // Initialize the clamp solenoid
        climbSolenoid = new Solenoid(PneumaticsModuleType.REVPH, HangConstants.climbSolenoid); // Initialize the climb solenoid
        compressor = new Compressor(PneumaticsModuleType.REVPH); // Initialize the compressor
        compressor.enableAnalog(HangConstants.minPressure, HangConstants.maxPressure); // Enable analog control with specified pressure limits
        
        // Log the initialization of the Hang subsystem
        logger.logData("Hang", "Initialization", "Hang subsystem initialized");
    }
    
    /**
     * Periodic method called regularly to update the state of the subsystem.
     * Logs the current state of the clamp and climb mechanisms to the SmartDashboard
     * and updates the logging system if any state changes occur.
     */
    @Override
    public void periodic() {
        // Log the activation state of the clamp and climb mechanisms
        SmartDashboard.putBoolean("Hang/Clamp Activated", isClampActivated()); // Display if the clamp is activated
        SmartDashboard.putBoolean("Hang/Climb Extended", isClimbExtended()); // Display if the climb is extended
        SmartDashboard.putNumber("Hang/Pressure", compressor.getPressure()); // Display the current pressure from the compressor
        
        // Call the method to log any state changes
        updateLogging(); // Update logging for state changes
    }
    
    /**
     * Updates the logging system with the current states of the solenoids and pressure.
     * Logs whether the clamp and climb mechanisms are activated and the current pressure.
     */
    private void updateLogging() {
        // Log the state of the clamp solenoid
        if (clampSolenoid.get()) {
            logger.logData("Hang", "Clamp State", "Activated"); // Log if the clamp is currently activated
        }
        // Log the state of the climb solenoid
        if (climbSolenoid.get()) {
            logger.logData("Hang", "Climb State", "Extended"); // Log if the climb is currently extended
        }
        
        // Log the current pressure from the compressor
        logger.logData("Hang", "Pressure", String.valueOf(compressor.getPressure())); // Log the current pressure reading
    }
    
    /**
     * Activates the clamp mechanism by setting the clamp solenoid to true.
     * Logs the activation command.
     */
    public void clampActivate() {
        clampSolenoid.set(true); // Activate the clamp solenoid
        logger.logData("Hang", "Command", "Clamp Activated"); // Log the command for activation
    }
    
    /**
     * Deactivates the clamp mechanism by setting the clamp solenoid to false.
     * Logs the deactivation command.
     */
    public void clampDeactivate() {
        clampSolenoid.set(false); // Deactivate the clamp solenoid
        logger.logData("Hang", "Command", "Clamp Deactivated"); // Log the command for deactivation
    }
    
    /**
     * Checks if the clamp mechanism is currently activated.
     * 
     * @return true if the clamp is activated, false otherwise.
     */
    public boolean isClampActivated() {
        return clampSolenoid.get(); // Return the current state of the clamp solenoid
    }
    
    /**
     * Extends the climbing mechanism by setting the climb solenoid to true.
     * Logs the extension command.
     */
    public void climbExtend() {
        climbSolenoid.set(true); // Extend the climb solenoid
        logger.logData("Hang", "Command", "Climb Extended"); // Log the command for extension
    }
    
    /**
     * Retracts the climbing mechanism by setting the climb solenoid to false.
     * Logs the retraction command.
     */
    public void climbRetract() {
        climbSolenoid.set(false); // Retract the climb solenoid
        logger.logData("Hang", "Command", "Climb Retracted"); // Log the command for retraction
    }
    
    /**
     * Checks if the climbing mechanism is currently extended.
     * 
     * @return true if the climb is extended, false otherwise.
     */
    public boolean isClimbExtended() {
        return climbSolenoid.get(); // Return the current state of the climb solenoid
    }
    
    /**
     * Checks if the subsystem is ready based on the compressor pressure.
     * 
     * @return true if the compressor pressure is above the minimum threshold, false otherwise.
     */
    @Override
    public boolean isReady() {
        return compressor.getPressure() >= HangConstants.minPressure; // Check if the pressure is sufficient
    }
    
    /**
     * Checks if the subsystem is at the setpoint.
     * For solenoids, this method always returns true since they are binary devices.
     * 
     * @return true, indicating that the solenoids are at their setpoint.
     */
    @Override
    public boolean atSetpoint() {
        return true; // Always return true for solenoid setpoint
    }
    
    /**
     * Retrieves the current measurement from the subsystem.
     * In this case, it returns the current pressure from the compressor.
     * 
     * @return the current pressure reading from the compressor.
     */
    @Override
    public double getMeasurement() {
        return compressor.getPressure(); // Return the current pressure from the compressor
    }
}

