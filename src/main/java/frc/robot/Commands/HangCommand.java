package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Hang;

public class HangCommand extends SequentialCommandGroup {
    public HangCommand(Hang hang_Subsystem){
        addCommands(

            Commands.runOnce(()-> hang_Subsystem.clampDeactivate()),

            new WaitCommand(1.0),
            
            Commands.runOnce(()-> hang_Subsystem.climbExtend()),

            new WaitCommand(1.0),

            Commands.runOnce(()-> hang_Subsystem.clampActivate()),

            new WaitCommand(1.0),

            Commands.runOnce(()-> hang_Subsystem.climbRetract())

        );
    }

    
}
