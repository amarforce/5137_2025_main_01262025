package frc.robot.Commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Wrist_Constants;
import frc.robot.Subsystems.wrist;


public class Wrist_Commands {
    private wrist wrist;
    private Timer timer;
    public Wrist_Commands(wrist wrist){
        this.wrist = wrist;
        timer = new Timer();
        timer.reset();
    }
    public InstantCommand stop(){
        return new InstantCommand(() -> wrist.stopWristMotor());
    }

    public InstantCommand wristForward(){
        if (wrist.wristPosition() == 90){
            return new InstantCommand(()-> wrist.wristPos1());
        }
            else{
            return new InstantCommand(()-> wrist.wristPos2());
        }
        
        //return new InstantCommand(() -> wrist.setWristSpeed(Wrist_Constants.defaultMotorSpeed), wrist);
    }
    public InstantCommand wristReverse(){
        return new InstantCommand(() -> wrist.setWristSpeed(-Wrist_Constants.defaultMotorSpeed), wrist);
    }



}
