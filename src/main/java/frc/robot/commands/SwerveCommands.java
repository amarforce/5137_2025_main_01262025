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

    public Command drive(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier fieldOriented) {
        return new InstantCommand(
            () -> swerve.setPercentDrive(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble(), fieldOriented.getAsBoolean()),
            swerve
        );
    }

    public Command driveToStation() {
        return new InstantCommand(
            () -> swerve.driveToPose(swerve.getClosest(SwerveConstants.stations)),
            swerve
        );
    }

    public Command driveToReefLeft() {
        return new InstantCommand(
            () -> swerve.driveToPose(swerve.getClosest(SwerveConstants.leftReef)),
            swerve
        );
    }

    public Command driveToReefCenter() {
        return new InstantCommand(
            () -> swerve.driveToPose(swerve.getClosest(SwerveConstants.centerReef)),
            swerve
        );
    }

    public Command driveToReefRight() {
        return new InstantCommand(
            () -> swerve.driveToPose(swerve.getClosest(SwerveConstants.rightReef)),
            swerve
        );
    }

    public Command driveToProcessor() {
        return new InstantCommand(
            () -> swerve.driveToPose(SwerveConstants.processor),
            swerve
        );
    }

    public Command driveToCage() {
        return new InstantCommand(
            () -> swerve.driveToPose(SwerveConstants.cages[swerve.getCage()]),
            swerve
        );
    }

    public Command lock() {
        return new InstantCommand(() -> swerve.lock(), swerve);
    }

    public Command resetGyro() {
        return new InstantCommand(() -> swerve.resetGyro(), swerve);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return swerve.getRoutine().quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return swerve.getRoutine().dynamic(direction);
    }
}
