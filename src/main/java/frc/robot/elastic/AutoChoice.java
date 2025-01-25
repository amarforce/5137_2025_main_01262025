package frc.robot.elastic;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.other.RobotUtils;

/**
 * Creates three {@link SendableChooser}s for level select, reef position, and pickup position.
 */
public class AutoChoice {

    private int id;

    private SendableChooser<Supplier<Command>> levelChooser;
    private SendableChooser<Pose2d> reefChooser;
    private SendableChooser<Pose2d> pickupChooser;

    public AutoChoice(int id, Supplier<Command>[] commands) {
        this.id = id;

        levelChooser = new SendableChooser<Supplier<Command>>();
        levelChooser.setDefaultOption("L4", commands[0]);
        levelChooser.addOption("L3", commands[1]);
        levelChooser.addOption("L2", commands[2]);
        levelChooser.addOption("L1", commands[3]);
        levelChooser.addOption("Algae", commands[4]);
        SmartDashboard.putData("Level Choice " + id, levelChooser);

        switchToCoralPoses();

        levelChooser.onChange((Supplier<Command> choice) -> {
            if (choice.equals(commands[4])) {
                switchToAlgaePoses();
            } else {
                switchToCoralPoses();
            }
        });
    }

    private void switchToCoralPoses() {
        reefChooser = new SendableChooser<Pose2d>();
        reefChooser.setDefaultOption("A", SwerveConstants.reefA);
        reefChooser.addOption("B", SwerveConstants.reefB);
        reefChooser.addOption("C", SwerveConstants.reefC);
        reefChooser.addOption("D", SwerveConstants.reefD);
        reefChooser.addOption("E", SwerveConstants.reefE);
        reefChooser.addOption("F", SwerveConstants.reefF);
        reefChooser.addOption("G", SwerveConstants.reefG);
        reefChooser.addOption("H", SwerveConstants.reefH);
        reefChooser.addOption("I", SwerveConstants.reefI);
        reefChooser.addOption("J", SwerveConstants.reefJ);
        reefChooser.addOption("K", SwerveConstants.reefK);
        reefChooser.addOption("L", SwerveConstants.reefL);
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
        reefChooser.setDefaultOption("AB", SwerveConstants.reefAB);
        reefChooser.addOption("CD", SwerveConstants.reefCD);
        reefChooser.addOption("EF", SwerveConstants.reefEF);
        reefChooser.addOption("GH", SwerveConstants.reefGH);
        reefChooser.addOption("IJ", SwerveConstants.reefIJ);
        reefChooser.addOption("KL", SwerveConstants.reefKL);
        SmartDashboard.putData("Reef Choice " + id, reefChooser);

        pickupChooser = new SendableChooser<Pose2d>();
        SmartDashboard.putData("Pickup Choice " + id, pickupChooser);
    }

    public Command getCommand() {
        return levelChooser.getSelected().get();
    }

    public Pose2d getPose() {
        return RobotUtils.invertPoseToAlliance(reefChooser.getSelected());
    }

    public Pose2d getPickup() {
        return RobotUtils.invertPoseToAlliance(pickupChooser.getSelected());
    }
}