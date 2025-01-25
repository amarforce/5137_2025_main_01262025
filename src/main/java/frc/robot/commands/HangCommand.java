package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.HangConstants;
import frc.robot.subsystems.Hang;

public class HangCommand extends SequentialCommandGroup {
    public HangCommand(Hang hangSubsystem){
        addCommands(
            Commands.runOnce(()-> hangSubsystem.clampDeactivate()),
            new WaitCommand(HangConstants.clampDeactivationTime),
            Commands.runOnce(()-> hangSubsystem.climbExtend()),
            new WaitCommand(HangConstants.climbExtensionTime),
            Commands.runOnce(()-> hangSubsystem.clampActivate()),
            new WaitCommand(HangConstants.clampActivationTime),
            Commands.runOnce(()-> hangSubsystem.climbRetract())
        );
    }

    
}
