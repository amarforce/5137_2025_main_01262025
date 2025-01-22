package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;


public class WristCommands {
    private Wrist wrist;
    
    public WristCommands(Wrist wrist){
        this.wrist = wrist;
    }

    public InstantCommand stop(){
        return new InstantCommand(() -> wrist.stopWristMotor());
    }

    public InstantCommand wristForward(){
        return new InstantCommand(()-> wrist.wristPos1());
    }

    public InstantCommand wristReverse(){
        return new InstantCommand(()-> wrist.wristPos2());
    }
}
