package frc.robot.other;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MultiCommands;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

/**
 * Creates three {@link SendableChooser}s for level select, reef position, and pickup position.
 */
public class AutoStep {

    private int id;

    private MultiCommands multiCommands;
    private SendableChooser<Integer> levelChooser;
    private SendableChooser<Pose2d> reefChooser;
    private SendableChooser<Pose2d> pickupChooser;

    public AutoStep(int id, MultiCommands multiCommands) {
        this.id = id;
        this.multiCommands = multiCommands;

        levelChooser = new SendableChooser<Integer>();
        levelChooser.setDefaultOption("L4", 4);
        levelChooser.addOption("L3", 3);
        levelChooser.addOption("L2", 2);
        levelChooser.addOption("L1", 1);
        levelChooser.addOption("Algae", 0);
        SmartDashboard.putData("Level Choice " + id, levelChooser);

        switchToCoralPoses();

        levelChooser.onChange((Integer choice) -> {
            if (choice.equals(0)) {
                switchToAlgaePoses();
            } else {
                switchToCoralPoses();
            }
        });
    }

    private void switchToCoralPoses() {
        reefChooser = new SendableChooser<Pose2d>();
        reefChooser.setDefaultOption("A", SwerveConstants.allReef[0]);
        for(int i=1;i<GeneralConstants.sides*2;i++){
            reefChooser.addOption(Character.toString('A'+i), SwerveConstants.allReef[i]);
        }
        SmartDashboard.putData("Reef Choice " + id, reefChooser);

        pickupChooser = new SendableChooser<Pose2d>();
        pickupChooser.setDefaultOption("RightStation Right", SwerveConstants.stationA);
        pickupChooser.addOption("RightStation Left", SwerveConstants.stationB);
        pickupChooser.addOption("LeftStation Right", SwerveConstants.stationC);
        pickupChooser.addOption("LeftStation Left", SwerveConstants.stationD);
        pickupChooser.addOption("Left Ground", SwerveConstants.leftPickup);
        pickupChooser.addOption("Center Ground", SwerveConstants.centerPickup);
        pickupChooser.addOption("Right Ground", SwerveConstants.rightPickup);
        SmartDashboard.putData("Pickup Choice " + id, pickupChooser);
    }

    private void switchToAlgaePoses() {
        reefChooser = new SendableChooser<Pose2d>();
        reefChooser.setDefaultOption("AB", SwerveConstants.centerReef[0]);
        for(int i=1;i<GeneralConstants.sides;i++){
            reefChooser.addOption(Character.toString('A'+(2*i))+Character.toString('A'+(2*i+1)), SwerveConstants.centerReef[i]);
        }
        SmartDashboard.putData("Reef Choice " + id, reefChooser);

        pickupChooser = new SendableChooser<Pose2d>();
        SmartDashboard.putData("Pickup Choice " + id, pickupChooser);
    }

    public Command getCommand() {
        return new SequentialCommandGroup(
            multiCommands.getCoral(pickupChooser.getSelected()),
            new ParallelCommandGroup(multiCommands.moveToGoal(levelChooser.getSelected()),multiCommands.getSwerveCommands().driveToPose(reefChooser.getSelected())));
    }

    public Pose2d getPose() {
        return RobotUtils.invertPoseToAlliance(reefChooser.getSelected());
    }

    public Pose2d getPickup() {
        return RobotUtils.invertPoseToAlliance(pickupChooser.getSelected());
    }
}