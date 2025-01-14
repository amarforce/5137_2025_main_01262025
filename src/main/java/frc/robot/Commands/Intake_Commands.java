package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Wrist_Constants;
import frc.robot.Subsystems.wrist;


public class Intake_Commands {
    private wrist intake;
    private Timer timer;

    public Intake_Commands(wrist intake){
        this.intake = intake;
        timer = new Timer();
        timer.reset();
    }
    public InstantCommand stop(){
        return new InstantCommand(() -> intake.stopIntakeMotor());
    }

    public InstantCommand wristForward(){
        return new InstantCommand(() -> intake.setIntakeSpeed(Wrist_Constants.defaultMotorSpeedIntake), intake);
    }
    public InstantCommand wristReverse(){
        return new InstantCommand(() -> intake.setIntakeSpeed(-Wrist_Constants.defaultMotorSpeedIntake), intake);
    }



}
