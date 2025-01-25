package frc.robot.other;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.SwerveConstants;

public class AutoFactory {
    private AutoStep choiceOne;
    private AutoStep choiceTwo;
    private AutoStep choiceThree;
    private AutoStep choiceFour;
    private AutoStep choiceFive;
    private Supplier<Command> groundIntake;
    private Supplier<Command> sourceIntake;
    
    private SendableChooser<Boolean> build;
    private PathPlannerAuto auto;

    @SuppressWarnings("unchecked")
    public AutoFactory(Supplier<Command>... commands) {
        choiceOne = new AutoStep(1, commands);
        choiceTwo = new AutoStep(2, commands);
        choiceThree = new AutoStep(3, commands);
        choiceFour = new AutoStep(4, commands);
        choiceFive = new AutoStep(5, commands);
        groundIntake = commands[5];
        sourceIntake = commands[6];

        build = new SendableChooser<Boolean>();
        build.setDefaultOption("AUTO NOT BUILT", false);
        build.addOption("AUTO BUILT", true);
        SmartDashboard.putData("Auto Builder", build);

        build.onChange((Boolean build) -> {
            if (build) {
                buildAuto();
            }
        });
    }

    public Command getCoral(Pose2d path) {
        if (path == null) {
            return new WaitCommand(0.0);
        } else {
            if (path.getY() > 1.75 && path.getY() < 6.3) {
                return new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(path, SwerveConstants.constraints),
                    groundIntake.get()
                );
            } else {
                return new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(path, SwerveConstants.constraints),
                    sourceIntake.get()
                );
            }
        }
    }

    public void buildAuto() {
        auto = new PathPlannerAuto(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceOne.getPose(), SwerveConstants.constraints),
                    choiceOne.getCommand()
                ),
                getCoral(choiceTwo.getPickup()),
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceTwo.getPose(), SwerveConstants.constraints),
                    choiceTwo.getCommand()
                ),
                getCoral(choiceThree.getPickup()),
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceThree.getPose(), SwerveConstants.constraints),
                    choiceThree.getCommand()
                ),
                getCoral(choiceFour.getPickup()),
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceFour.getPose(), SwerveConstants.constraints),
                    choiceFour.getCommand()
                ),
                getCoral(choiceFive.getPickup()),
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceFive.getPose(), SwerveConstants.constraints),
                    choiceFive.getCommand()
                )
            )
        );
    }

    public PathPlannerAuto getAuto() {
        if (auto == null) {
            buildAuto();
        }
        return auto;
    }
}
