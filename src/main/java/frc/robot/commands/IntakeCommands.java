package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.constants.IntakeConstants;

public class IntakeCommands {
    private Intake intake;

    public IntakeCommands(Intake intake){
        this.intake = intake;
    }
    public InstantCommand stop(){
        return new InstantCommand(() -> intake.stop());
    }

    public InstantCommand intakeForward(){
        return new InstantCommand(() -> intake.setSpeed(IntakeConstants.defaultMotorSpeedIntake), intake);
    }
    public InstantCommand intakeReverse(){
        return new InstantCommand(() -> intake.setSpeed(-IntakeConstants.defaultMotorSpeedIntake), intake);
    }
}
