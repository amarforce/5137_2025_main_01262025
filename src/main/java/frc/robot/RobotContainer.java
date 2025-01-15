// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.*;
import frc.robot.Commands.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;



public class RobotContainer {

  private CommandPS5Controller driver;
  private CommandPS5Controller operator;

  private wrist wrist;
  private intake intake;

  private Wrist_Commands Wrist_Commands;
  private Intake_Commands Intake_Commands;

  



  public RobotContainer() {
    driver = new CommandPS5Controller(0);
    operator = new CommandPS5Controller(1);

    configureBindings();
    wrist = new wrist();
    intake = new intake();

    Wrist_Commands = new Wrist_Commands(wrist);
    Intake_Commands = new Intake_Commands(intake);
  }

  private void configureBindings() {
    driver.L2()
    .onTrue(Intake_Commands.intakeForward())
    .onFalse(Intake_Commands.stop()
    );

    operator.L2()
    .onTrue(Wrist_Commands.wristForward())
    .onFalse(Wrist_Commands.stop()
    );
    // No reverse created
    //Talk to mechanical about wrist positions

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
