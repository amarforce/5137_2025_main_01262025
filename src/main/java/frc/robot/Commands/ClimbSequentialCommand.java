package frc.robot.Commands;
import frc.robot.Constants.Hang_Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Hang_Subsystem;

public class ClimbSequentialCommand extends SequentialCommandGroup {
    public ClimbSequentialCommand(Hang_Subsystem hang_Subsystem){
        addCommands(
            Commands.runOnce(()-> hang_Subsystem.climbExtend()),

            Commands.runOnce(()-> hang_Subsystem.clampDeactivate())
        );
    }

    
}
