package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Arm;
import frc.robot.constants.ArmConstants;


public class ArmCommands {

    private Arm arm;

    public ArmCommands(Arm arm) {
        this.arm = arm;
    }
    
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> arm.setGoal(goal.getAsDouble()), arm);
    }

    public Command changeGoal(DoubleSupplier change){
        return new InstantCommand(() -> arm.setGoal(arm.getGoal()+change.getAsDouble()), arm);
    }

    public Command moveToGoal(int goal) {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.goals[goal-1]), arm);
    }

    public Command moveToSource() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.sourceGoal), arm);
    }

    public Command moveToGroundIntake() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.groundIntakeGoal), arm);
    }

    public Command moveToDefault(){
        return new InstantCommand(()->arm.setGoal(ArmConstants.defaultGoal), arm);
    }

    public Command moveToAlgae(){
        return new InstantCommand(()->arm.setGoal(ArmConstants.algaeGoal), arm);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir){
        return arm.sysIdRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir){
        return arm.sysIdRoutine.dynamic(dir);
    }
}
