// src/main/java/frc/robot/commands/recipes/coral/DropCoralL1Command.java
package frc.robot.commands.recipes.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MultiCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.monitoring.RecipeMonitor;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.WristConstants;
import frc.robot.utils.logging.CoralLogger;

public class DropCoralL1Command extends SequentialCommandGroup {
    private final CoralLogger logger = new CoralLogger();
    private final MultiCommands multiCommands;
    private final SwerveCommands swerveCommands;
    private final IntakeCommands intakeCommands;
    private final RecipeMonitor recipeMonitor;

    /**
     * Creates a new command for dropping coral on L1 level.
     * This command coordinates multiple subsystems to accurately place coral at the lowest level.
     *
     * @param multiCommands Combined subsystem commands for elevator, arm, and wrist control.
     * @param swerveCommands Drive system commands for robot movement and positioning.
     * @param intakeCommands Intake/outtake control commands for handling coral.
     * @param recipeMonitor Monitor for tracking the progress and status of recipe commands.
     */
    public DropCoralL1Command(
        RecipeMonitor recipeMonitor,
        MultiCommands multiCommands,
        SwerveCommands swerveCommands,
        IntakeCommands intakeCommands) {
        
        this.multiCommands = multiCommands;
        this.swerveCommands = swerveCommands;
        this.intakeCommands = intakeCommands;

        logger.logPlacementStart(1);  // Log start of L1 placement
        
        addCommands(
            
            // Step 0: Recipe monitoring initialization
            Commands.runOnce(() -> {
                // Start tracking the "DropCoralL1" recipe execution.
                recipeMonitor.startRecipe("DropCoralL1");
            }),

            // Step 1: Safety checks and initial setup
            Commands.runOnce(() -> {
                // Verify all subsystems are operational and safe before proceeding.
                if (!isSystemSafe()) {
                    // Log an error if any subsystem fails the safety check.
                    logger.logError("SUBSYSTEM_NOT_READY", "Systems failed safety check for L1 placement");
                    // Abort command execution by throwing an exception if safety checks fail.
                    throw new RuntimeException("Subsystems not ready for L1 placement");
                }
                // Log the initial positions of the elevator, arm, and wrist for debugging and analysis.
                logger.logPositioning(
                    multiCommands.getElevator().getMeasurement(),
                    multiCommands.getArm().getMeasurement(),
                    multiCommands.getWrist().getMeasurement()
                );
            }),

            // Step 2: Update recipe monitor to indicate approaching reef
            Commands.runOnce(() -> {
                recipeMonitor.updateStep("DropCoralL1", "Approaching reef");
            }),
            // Approach commands will be added here in future if needed for L1. Currently L1 placement assumes robot is close enough.
            
            // Step 3: Update recipe monitor to indicate positioning phase
            Commands.runOnce(() -> {
                recipeMonitor.updateStep("DropCoralL1", "Positioning");
            }),
            // Positioning commands will be added here in future if needed for L1. Currently L1 placement assumes direct placement.
            
            // Step 4: Update recipe monitor to indicate coral placement phase
            Commands.runOnce(() -> {
                recipeMonitor.updateStep("DropCoralL1", "Placing coral");
            }),
            // Placement commands are executed in the following steps.
            
            // Step 5: Recipe completion logging (initial call, status updated later)
            Commands.runOnce(() -> {
                recipeMonitor.completeRecipe("DropCoralL1", true); // Initial complete call, status might be updated in 'end' method.
            }),

            // Step 6: Simultaneously drive to reef center and move mechanisms to L1 target position
            new ParallelCommandGroup(
                // Command to drive the robot to the center of the reef, optimizing position for L1 drop.
                swerveCommands.driveToReefCenter(),
                // Command to move the elevator, arm, and wrist to their predefined L1 target positions concurrently.
                multiCommands.moveToGoal(1)
            ).withTimeout(3.0)  // Set a 3-second timeout to prevent indefinite waiting if approach fails.
            .handleInterrupt(() -> {
                // If approach is interrupted (e.g., by user or fault), retract mechanisms to default safe positions.
                multiCommands.moveToDefault();
                // Log an error indicating the approach movement timed out, suggesting a potential issue.
                logger.logError("APPROACH_TIMEOUT", "Approach movement timed out");
            }),

            // Step 7: Lock swerve drive and ensure wrist is in correct orientation for L1 placement
            new ParallelCommandGroup(
                // Command to engage swerve drive locks to maintain robot's position during coral placement.
                swerveCommands.lock(),
                // Command to orient the wrist to the predefined position suitable for L1 coral placement.
                Commands.runOnce(() -> {
                    multiCommands.getWrist().toPos1(); // Ensure wrist is correctly positioned for L1 using preset position 1.
                })
            ).withTimeout(1.0), // Set a 1-second timeout for final positioning to prevent indefinite waiting.

            // Step 8: Short pause to allow mechanisms to settle before outtake
            new WaitCommand(0.1), // Introduce a 0.1-second pause to ensure mechanisms are stable before releasing coral.

            // Step 9: Execute coral placement sequence
            new SequentialCommandGroup(
                // Command to activate outtake mechanism briefly to gently release the coral onto L1.
                intakeCommands.outtake().withTimeout(0.3), // Use a short 0.3-second outtake duration for controlled L1 placement.
                // Command to immediately stop the intake motors after coral is released to prevent accidental re-intake.
                intakeCommands.stop()
            ),

            // Step 10: Safe retraction of mechanisms and maintain robot position
            new ParallelCommandGroup(
                // Command to retract elevator, arm, and wrist to their default safe positions simultaneously.
                multiCommands.moveToDefault(),
                // Command to keep the swerve drive locked during retraction to maintain robot's position.
                swerveCommands.lock()
            ).withTimeout(2.0) // Set a 2-second timeout for retraction to prevent indefinite waiting.
            .handleInterrupt(() -> {
                // If retraction is interrupted, log an error indicating retraction movement timed out.
                logger.logError("RETRACTION_TIMEOUT", "Retraction movement timed out");
            }),

            // Step 11: Final logging of L1 placement completion
            Commands.runOnce(() -> {
                // Log the successful completion of the coral placement operation at L1.
                logger.logPlacementComplete(1, true); // Log success status for L1 placement.
            })
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            recipeMonitor.reportError("DropCoralL1", "Command interrupted");
            recipeMonitor.completeRecipe("DropCoralL1", false);
        }
        super.end(interrupted);
    }

    /**
     * Checks if all systems are within safe operating parameters for L1 placement.
     * This includes checking elevator, arm, and wrist limits and ensuring the elevator is not at its lower limit.
     *
     * @return true if all systems are safe for L1 placement, false otherwise.
     */
    private boolean isSystemSafe() {
        return multiCommands.getElevator().isWithinLimits() &&
               multiCommands.getArm().isWithinLimits() &&
               multiCommands.getWrist().isWithinLimits() &&
               !multiCommands.getElevator().isAtLowerLimit(); // Extra safety check to ensure elevator is not at lower limit for L1
    }

    /**
     * Utility method to check if the robot's elevator is at the expected L1 height.
     * Compares the current elevator height to the predefined L1 height from VisionConstants.
     *
     * @param currentHeight The current height of the elevator as measured.
     * @return true if the current height is within the tolerance of the L1 height, false otherwise.
     */
    private boolean isAtL1Height(double currentHeight) {
        return Math.abs(currentHeight - VisionConstants.l1_height) <
               ElevatorConstants.elevatorTol; // Compare current height to L1 height with tolerance
    }
}