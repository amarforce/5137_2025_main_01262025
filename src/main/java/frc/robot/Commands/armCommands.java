package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;
import frc.robot.Subsystems.Arm;


public class armCommands{
     Arm arm;
        public armCommands(Arm arm) {
            this.arm = arm;
        }
   
        public Command setSpeed(DoubleSupplier speed) {
            return new InstantCommand(()-> arm.setSpeed(speed.getAsDouble()), arm);
        }
        public Command setGoal(DoubleSupplier goal) {
            return new InstantCommand(() -> arm.setGoal(goal.getAsDouble()), arm);
        }


        //set to level commands

        public Command move2L1() {
            return new InstantCommand(() -> arm.setGoal(armConstants.goal1), arm);
        }
        public Command move2L2() {
            return new InstantCommand(() -> arm.setGoal(armConstants.goal2), arm);
        }
        public Command move2L3() {
            return new InstantCommand(() -> arm.setGoal(armConstants.goal3), arm);
        }
        public Command move2L4() {
            return new InstantCommand(() -> arm.setGoal(armConstants.goal4), arm);
        }
}
