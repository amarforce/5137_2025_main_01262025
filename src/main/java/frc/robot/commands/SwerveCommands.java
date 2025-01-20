package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class SwerveCommands {
    private Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    public InstantCommand drive(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier fieldOriented) {
        return new InstantCommand(
            () -> swerve.percentOutput(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble(), fieldOriented.getAsBoolean()),
            swerve
        );
    }

    public InstantCommand driveToReefLeft() {
        return new InstantCommand (
            () -> swerve.driveToPose(SwerveConstants.reefA),
            swerve
        );
    }

    public InstantCommand driveToReefCenter() {
        return new InstantCommand (
            () -> swerve.driveToPose(SwerveConstants.reefAB),
            swerve
        );
    }

    public InstantCommand driveToReefRight() {
        return new InstantCommand (
            () -> swerve.driveToPose(SwerveConstants.reefB),
            swerve
        );
    }

    public InstantCommand lock() {
        return new InstantCommand(() -> swerve.lock(), swerve);
    }

    public InstantCommand resetGyro() {
        return new InstantCommand(() -> swerve.resetGyro(), swerve);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return swerve.getRoutine().quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return swerve.getRoutine().dynamic(direction);
    }
}
