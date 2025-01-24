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

    public Command setSpeed(DoubleSupplier speed){
        return new InstantCommand(()->elevator.setSpeed(speed.getAsDouble()),elevator);
    }

    public Command setGoal(DoubleSupplier goal){
        return new InstantCommand(()->elevator.setGoal(goal.getAsDouble()),elevator);
    }

    public Command moveToL1(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.goal1),elevator);
    }
    
    public Command moveToL2(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.goal2),elevator);
    }

    public Command moveToL3(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.goal3),elevator);
    }

    public Command moveToL4(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.goal4),elevator);
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir){
        return elevator.sysIdRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir){
        return elevator.sysIdRoutine.dynamic(dir);
    }
}
