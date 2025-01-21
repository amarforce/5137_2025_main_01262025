package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    public int getClosestSide() {
        Pose2d pose = swerve.getPose();
        if (swerve.onRedAlliance()) {
            pose = swerve.invertPose(pose);
        }
        int closest = 0;
        double closest_distance = 100.0;
        for (int i = 0; i < 6; i++) {
            Transform2d transform = pose.minus(SwerveConstants.centerReef[i]);
            double distance = Math.hypot(transform.getX(), transform.getY());
            if (distance < closest_distance) {
                closest_distance = distance;
                closest = i;
            }
        }
        return closest;
    }

    public InstantCommand driveToReefLeft() {
        return new InstantCommand (
            () -> swerve.driveToPose(SwerveConstants.leftReef[getClosestSide()]),
            swerve
        );
    }

    public InstantCommand driveToReefCenter() {
        return new InstantCommand (
            () -> swerve.driveToPose(SwerveConstants.centerReef[getClosestSide()]),
            swerve
        );
    }

    public InstantCommand driveToReefRight() {
        return new InstantCommand (
            () -> swerve.driveToPose(SwerveConstants.rightReef[getClosestSide()]),
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
