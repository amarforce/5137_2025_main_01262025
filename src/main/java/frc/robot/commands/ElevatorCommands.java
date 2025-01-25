package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommands {
    Elevator elevator;
    public ElevatorCommands(Elevator elevator){
        this.elevator = elevator;
    }

    public Command setGoal(DoubleSupplier goal){
        return new InstantCommand(()->elevator.setGoal(goal.getAsDouble()),elevator);
    }

    public Command changeGoal(DoubleSupplier change){
        return new InstantCommand(()->elevator.setGoal(elevator.getGoal()+change.getAsDouble()),elevator);
    }

    public Command moveToGoal(int goal){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.goals[goal-1]),elevator);
    }

    public Command moveToSource(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.sourceGoal),elevator);
    }

    public Command moveToGroundIntake(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.groundIntakeGoal),elevator);
    }

    public Command moveToDefault(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.defaultGoal),elevator);
    }

    public Command moveToAlgae(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.algaeGoal),elevator);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir){
        return elevator.sysIdRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir){
        return elevator.sysIdRoutine.dynamic(dir);
    }
}
