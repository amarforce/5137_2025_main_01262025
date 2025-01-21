// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
    private final CommandPS5Controller driver = new CommandPS5Controller(0);

    private Vision vision;
    private Swerve swerve;

    private SwerveCommands swerveCommands;

    public RobotContainer() {

      vision = new Vision();
      swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve.json"), vision);

      swerveCommands = new SwerveCommands(swerve);

      configureBindings();
    }

    private void configureBindings() {
        // Driver Bindings

        swerve.setDefaultCommand(
            swerveCommands.drive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> driver.R1().negate().getAsBoolean())
        );

        driver.cross().whileTrue(swerveCommands.lock());

        driver.triangle().onTrue(swerveCommands.driveToStation());
        driver.square().onTrue(swerveCommands.driveToCage());
        driver.circle().onTrue(swerveCommands.driveToProcessor());

        driver.povLeft().onTrue(swerveCommands.driveToReefLeft());
        driver.povUp().onTrue(swerveCommands.driveToReefCenter());
        driver.povRight().onTrue(swerveCommands.driveToReefRight());

        driver.options().onTrue(swerveCommands.resetGyro());

        /*
        driver.povUp().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineTranslation)));
        driver.povLeft().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineSteer)));
        driver.povRight().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineRotation)));
        driver.options().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
        driver.options().and(driver.povDown()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
        driver.create().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
        driver.create().and(driver.povDown()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
        */

        driver.touchpad().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
