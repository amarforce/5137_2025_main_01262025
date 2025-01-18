// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.Subsystems.*;
import frc.robot.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;



public class RobotContainer {

  private boolean cat = true;


  private CommandPS5Controller driver;
  private CommandPS5Controller operator;

  private wrist wrist;
  private intake intake;

  private Wrist_Commands wrist_Commands;
  private Intake_Commands intake_Commands;

    



  public RobotContainer() {
    driver = new CommandPS5Controller(0);
    operator = new CommandPS5Controller(1);

 
    wrist = new wrist();
    intake = new intake();

    wrist_Commands = new Wrist_Commands(wrist);
    intake_Commands = new Intake_Commands(intake);
    configureBindings();
  }

  

  private void configureBindings() {
    

  operator.L2()
  .onTrue(wrist_Commands.wristForward());

  operator.L1()
      .onTrue(wrist_Commands.wristReverse());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
