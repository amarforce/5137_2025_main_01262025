package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Wrist;


public class WristCommands {
    private Wrist wrist;
    
    public WristCommands(Wrist wrist){
        this.wrist = wrist;
    }

    public Command toPos1(){
        return new InstantCommand(()-> wrist.setGoal(WristConstants.pos1));
    }

    public Command toPos2(){
        return new InstantCommand(()-> wrist.setGoal(WristConstants.pos2));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir){
        return wrist.sysIdRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir){
        return wrist.sysIdRoutine.dynamic(dir);
    }
}
