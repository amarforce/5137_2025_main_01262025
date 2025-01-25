package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.constants.IntakeConstants;

public class IntakeCommands {
    private Intake intake;

    public IntakeCommands(Intake intake){
        this.intake = intake;
    }

    public Command stop(){
        return new InstantCommand(() -> intake.stop());
    }

    public Command intakeUntilSwitched(){
        return new FunctionalCommand(()->intake.setSpeed(IntakeConstants.intakeSpeed), ()->{}, (e)->{}, ()->intake.isSwitched(), intake);
    }

    public Command outtake(){
        return new SequentialCommandGroup(Commands.runOnce(()->intake.setSpeed(IntakeConstants.intakeSpeed)),new WaitCommand(1),stop());
    }
}
