// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Commands.ClimbSequentialCommand;
import frc.robot.Subsystems.Hang_Subsystem;




public class RobotContainer {
  public RobotContainer() {
      configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
public class RobotContainer {
  
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(0); // Controller on port 0

  
  private final Hang_Subsystem hangSubsystem = new Hang_Subsystem();

  public RobotContainer() {
      
      configureBindings();
  }

  private void configureBindings() {
      
      ps5Controller.touchpad().onTrue(new ClimbSequentialCommand(hangSubsystem));

      
      ps5Controller.create().onTrue(new ClimbSequentialCommand(hangSubsystem));
  }

  public Command getAutonomousCommand() {
      
      return null;
  }
}
