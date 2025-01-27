package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.WristConstants;

public class DropCoralL4Command extends SequentialCommandGroup {
    
    /**
     * Creates a new command for dropping coral on L4 level.
     * This command coordinates multiple subsystems to accurately place coral.
     *
     * @param multiCommands Combined subsystem commands
     * @param swerveCommands Drive system commands
     * @param intakeCommands Intake/outtake control commands
     */
    public DropCoralL4Command(
            MultiCommands multiCommands,
            SwerveCommands swerveCommands,
            IntakeCommands intakeCommands) {
        
        addCommands(
            // Step 1: Initial safety check and preparation
            Commands.runOnce(() -> {
                // Verify all subsystems are operational
                if (!multiCommands.areSubsystemsReady()) {
                    throw new RuntimeException("Subsystems not ready for L4 placement");
                }
            }),

            // Step 2: Vision-guided approach to reef
            new ParallelCommandGroup(
                // Drive to reef center while preparing systems
                swerveCommands.driveToReefCenter(),
                // Pre-position elevator to intermediate height for safety
                multiCommands.moveToGoal(3) // Use L3 height during approach
            ),

            // Step 3: Final positioning and height adjustment
            new ParallelCommandGroup(
                // Lock swerve drive in position
                swerveCommands.lock(),
                // Move to final L4 position (height: 1.6m)
                multiCommands.moveToGoal(4),
                // Ensure wrist is in correct orientation
                Commands.runOnce(() -> {
                    // Using pos2 (90 degrees) for straight placement
                    wristCommands.toPos2()
                })
            ),

            // Step 4: Verification pause
            new WaitCommand(0.2), // Short pause to ensure stability

            // Step 5: Place coral
            new SequentialCommandGroup(
                // Gentle release of coral
                intakeCommands.outtake().withTimeout(0.5), // Half second release
                // Stop intake motors
                intakeCommands.stop()
            ),

            // Step 6: Safe retraction
            new ParallelCommandGroup(
                // Return to default positions
                multiCommands.moveToDefault(),
                // Maintain position until arm is clear
                swerveCommands.lock()
            )
        );
    }

    /**
     * Utility method to check if the robot is in position for L4 placement
     * @param currentHeight Current elevator height
     * @param targetHeight Target L4 height
     * @return boolean indicating if height is within tolerance
     */
    private boolean isAtL4Height(double currentHeight, double targetHeight) {
        return Math.abs(currentHeight - targetHeight) < ElevatorConstants.elevatorTol;
    }


}