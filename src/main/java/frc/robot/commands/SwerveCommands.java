package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
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

    public Command drive(DoubleSupplier dx, DoubleSupplier dy, DoubleSupplier dtheta, BooleanSupplier fieldOriented) {
        return new InstantCommand(
            () -> swerve.setPercentDrive(dx.getAsDouble(), dy.getAsDouble(), dtheta.getAsDouble(), fieldOriented.getAsBoolean()),
            swerve
        );
    }

    public Command driveToPose(Supplier<Pose2d> pose){
        Command auto=AutoBuilder.pathfindToPose(pose.get(), SwerveConstants.constraints);
        auto.addRequirements(swerve);
        return auto;
    }

    public Command driveToStation() {
        return driveToPose(()->swerve.getClosest(SwerveConstants.stations));
    }

    public Command driveToReefLeft() {
        return driveToPose(()->swerve.getClosest(SwerveConstants.leftReef));
    }

    public Command driveToReefCenter() {
        return driveToPose(()->swerve.getClosest(SwerveConstants.centerReef));
    }

    public Command driveToReefRight() {
        return driveToPose(()->swerve.getClosest(SwerveConstants.rightReef));
    }

    public Command driveToProcessor() {
        return driveToPose(()->SwerveConstants.processor);
    }

    public Command driveToCage() {
        return driveToPose(()->SwerveConstants.cages[swerve.getCage()]);
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
