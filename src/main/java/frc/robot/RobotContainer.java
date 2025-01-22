// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
  private CommandPS5Controller driver;
  private CommandPS5Controller operator;

  private Wrist wrist;
  private Intake intake;

  private WristCommands wristCommands;
  private IntakeCommands intakeCommands;

  public RobotContainer() {
    driver = new CommandPS5Controller(0);
    operator = new CommandPS5Controller(1);
 
    wrist = new Wrist();
    intake = new Intake();

    wristCommands = new WristCommands(wrist);
    intakeCommands = new IntakeCommands(intake);

    configureBindings();
  }

  private void configureBindings() {
  operator.R1()
  .onTrue(wristCommands.wristForward())
  .onFalse(wristCommands.wristReverse());

  operator.L2()
  .onTrue(intakeCommands.intakeReverse());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
