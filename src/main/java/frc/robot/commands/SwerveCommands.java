package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

    public InstantCommand lock() {
        return new InstantCommand(() -> swerve.lock(), swerve);
    }

    public InstantCommand reset() {
        return new InstantCommand(() -> swerve.reset(), swerve);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return swerve.getRoutine().quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return swerve.getRoutine().dynamic(direction);
    }
}
