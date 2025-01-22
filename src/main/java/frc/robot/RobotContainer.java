package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.elastic.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
  private CommandPS5Controller driver;
  private CommandPS5Controller operator;

  private Vision vision;
  private Swerve swerve;
  private Elevator elevator;

  private SwerveCommands swerveCommands;
  private ElevatorCommands elevatorCommands;

  private Reef reef;

  public RobotContainer() {
    driver = new CommandPS5Controller(0);
    operator = new CommandPS5Controller(1);

    vision = new Vision();
    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve.json"), vision);
    elevator = new Elevator();

    swerveCommands = new SwerveCommands(swerve);
    elevatorCommands = new ElevatorCommands(elevator);

    reef = new Reef();
    SmartDashboard.putData("Reef", reef);

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

    // Operator Bindings

    //elevator.setManualControl(true);
    elevator.setDefaultCommand(elevatorCommands.setGoal(()->1-operator.getLeftY()));

    operator.triangle().onTrue(elevatorCommands.moveToL4());
    operator.circle().onTrue(elevatorCommands.moveToL3());
    operator.square().onTrue(elevatorCommands.moveToL2());
    operator.cross().onTrue(elevatorCommands.moveToL1());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
