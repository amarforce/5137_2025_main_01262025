// src/main/java/frc/robot/commands/MultiCommands.java
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class MultiCommands {
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Swerve swerve;
    private final Intake intake;
    private final Hang hang;
    private final ArmCommands armCommands;
    private final ElevatorCommands elevatorCommands;
    private final WristCommands wristCommands;
    private final SwerveCommands swerveCommands;
    private final IntakeCommands intakeCommands;
    private final HangCommand hangCommand;

    public MultiCommands(
            Arm arm,
            Elevator elevator,
            Wrist wrist,
            Swerve swerve,
            Intake intake,
            Hang hang,
            ArmCommands armCommands,
            ElevatorCommands elevatorCommands,
            WristCommands wristCommands,
            SwerveCommands swerveCommands,
            IntakeCommands intakeCommands,
            HangCommand hangCommand) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
        this.swerve = swerve;
        this.intake = intake;
        this.hang = hang;
        this.armCommands = armCommands;
        this.elevatorCommands = elevatorCommands;
        this.wristCommands = wristCommands;
        this.swerveCommands = swerveCommands;
        this.intakeCommands = intakeCommands;
        this.hangCommand = hangCommand;
    }

    // Getter methods for subsystems
    // Returns the elevator subsystem
    public Elevator getElevator() {
        return elevator;
    }

    // Returns the arm subsystem
    public Arm getArm() {
        return arm;
    }

    // Returns the wrist subsystem
    public Wrist getWrist() {
        return wrist;
    }

    // Returns the swerve subsystem
    public Swerve getSwerve() {
        return swerve;
    }

    // Returns the intake subsystem
    public Intake getIntake() {
        return intake;
    }

    // Returns the hang subsystem
    public Hang getHang() {
        return hang;
    }

    // Command methods
    /**
     * Moves the robot's arm, elevator, and wrist to the ground intake position.
     * This is a convenience command that combines the individual subsystem commands
     * {@link ArmCommands#moveToGroundIntake()}, {@link ElevatorCommands#moveToGroundIntake()}, and {@link WristCommands#toPos1()}.
     * @return A {@link ParallelCommandGroup} that runs the commands simultaneously.
     */
    public Command moveToGroundIntake() {
        return new ParallelCommandGroup(
            armCommands.moveToGroundIntake(),
            elevatorCommands.moveToGroundIntake(),
            wristCommands.toPos1()
        );
    }

    /**
     * Moves the robot's arm, elevator, and wrist to their default positions.
     * This is a convenience command that combines the individual subsystem commands
     * {@link ArmCommands#moveToDefault()}, {@link ElevatorCommands#moveToDefault()}, and {@link WristCommands#toPos1()}.
     * @return A {@link ParallelCommandGroup} that runs the commands simultaneously.
     */
    public Command moveToDefault() {
        return new ParallelCommandGroup(
            armCommands.moveToDefault(),
            elevatorCommands.moveToDefault(),
            wristCommands.toPos1()
        );
    }

    /**
     * Moves the robot's arm, elevator, and wrist to the source position.
     * This is a convenience command that combines the individual subsystem commands
     * {@link ArmCommands#moveToSource()}, {@link ElevatorCommands#moveToSource()}, and {@link WristCommands#toPos2()}.
     * @return A {@link ParallelCommandGroup} that runs the commands simultaneously.
     */
    public Command moveToSource() {
        return new ParallelCommandGroup(
            armCommands.moveToSource(),
            elevatorCommands.moveToSource(),
            wristCommands.toPos2()
        );
    }

    /**
     * Moves the robot's arm, elevator, and wrist to the algae position.
     * This is a convenience command that combines the individual subsystem commands
     * {@link ArmCommands#moveToAlgae()}, {@link ElevatorCommands#moveToAlgae()}, and {@link WristCommands#toPos2()}.
     * @return A {@link ParallelCommandGroup} that runs the commands simultaneously.
     */
    public Command moveToAlgae() {
        return new ParallelCommandGroup(
            armCommands.moveToAlgae(),
            elevatorCommands.moveToAlgae(),
            wristCommands.toPos2()
        );
    }

    /**
     * Moves the robot's arm, elevator, and wrist to a specified goal position.
     * This is a convenience command that combines the individual subsystem commands
     * {@link ArmCommands#moveToGoal(int)}, {@link ElevatorCommands#moveToGoal(int)}, and {@link WristCommands#toPos2()}.
     * @param goal The goal level (e.g., 1, 2, or 3).
     * @return A {@link ParallelCommandGroup} that runs the commands simultaneously.
     */
    public Command moveToGoal(int goal) {
        return new ParallelCommandGroup(
            armCommands.moveToGoal(goal),
            elevatorCommands.moveToGoal(goal),
            wristCommands.toPos2()
        );
    }

    /**
     * Checks if all required subsystems are ready for operation.
     * This method checks the readiness of the {@link Elevator}, {@link Arm}, {@link Wrist}, and {@link Intake} subsystems
     * by calling their respective `isReady()` methods.
     * @return true if all subsystems are ready, false otherwise.
     */
    public boolean areSubsystemsReady() {
        return elevator.isReady() &&
               arm.isReady() &&
               wrist.isReady() &&
               intake.isReady();
    }

    /**
     * Gets a command to navigate to a coral (game piece) based on its pose.
     * If the provided pose is null, it returns an empty {@link InstantCommand}.
     * If the Y-coordinate of the pose indicates it's in a "close" range (1.75 to 6.3), it moves to the ground intake position after driving to the pose.
     * Otherwise, it moves to the source position after driving to the pose.
     * @param pose The {@link Pose2d} of the coral.
     * @return A {@link Command} to get the coral, combining swerve driving and arm/elevator/wrist movements.
     */
    public Command getCoral(Pose2d pose) {
        if (pose == null) {
            return new InstantCommand();
        } else {
            if (pose.getY() > 1.75 && pose.getY() < 6.3) {
                return new ParallelCommandGroup(
                    swerveCommands.driveToPose(()->pose),
                    moveToGroundIntake()
                );
            } else {
                return new ParallelCommandGroup(
                    swerveCommands.driveToPose(()->pose),
                    moveToSource()
                );
            }
        }
    }

    /**
     * Returns the SwerveCommands object, providing access to swerve-related commands.
     * @return The {@link SwerveCommands} object associated with this MultiCommands instance.
     */
    public SwerveCommands getSwerveCommands() {
        return swerveCommands;
    }
}
