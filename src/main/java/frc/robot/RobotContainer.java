// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.ArmCommands;
import frc.robot.subsystems.*;

public class RobotContainer {

	private Arm arm;
	private CommandPS5Controller driver;
	private CommandPS5Controller operator;
	private ArmCommands armCommands;

	public RobotContainer() {
		
		arm = new Arm();
		driver = new CommandPS5Controller(0);
		operator = new CommandPS5Controller(1);
		armCommands = new ArmCommands(arm);
		configureBindings();
	}

	private void configureBindings() {
		operator.triangle().onTrue(armCommands.moveToL4());
		operator.square().onTrue(armCommands.moveToL3());
		operator.circle().onTrue(armCommands.moveToL2());
		operator.cross().onTrue(armCommands.moveToL1());
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
