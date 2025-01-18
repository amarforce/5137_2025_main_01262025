package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.wristConstants;
import frc.robot.Subsystems.intake;



public class intakeCommands {
    private intake intake;
    private Timer timer;

    public intakeCommands(intake intake){
        this.intake = intake;
        timer = new Timer();
        timer.reset();
    }
    public InstantCommand stop(){
        return new InstantCommand(() -> intake.stopIntakeMotor());
    }

    public InstantCommand intakeForward(){
        return new InstantCommand(() -> intake.setIntakeSpeed(wristConstants.defaultMotorSpeedIntake), intake);
    }
    public InstantCommand intakeReverse(){
        return new InstantCommand(() -> intake.setIntakeSpeed(-wristConstants.defaultMotorSpeedIntake), intake);
    }



}
