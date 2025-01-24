package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public Command moveToL1() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.goal1), arm);
    }

    public Command moveToL2() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.goal2), arm);
    }

    public Command moveToL3() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.goal3), arm);
    }

    public Command moveToL4() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.goal4), arm);
    }
}
