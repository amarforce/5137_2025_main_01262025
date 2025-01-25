package frc.robot.other;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MultiCommands;
import frc.robot.constants.GeneralConstants;


public class AutoFactory {
    private AutoStep[] choices;
    private SendableChooser<Boolean> build;
    private PathPlannerAuto auto;

    public AutoFactory(MultiCommands multiCommands) {
        choices=new AutoStep[GeneralConstants.numAuto];

        build = new SendableChooser<Boolean>();
        build.setDefaultOption("AUTO NOT BUILT", false);
        build.addOption("AUTO BUILT", true);
        SmartDashboard.putData("Auto Builder", build);

        for(int i=0;i<GeneralConstants.numAuto;i++){
            choices[i]=new AutoStep(i+1, multiCommands);
        }

        build.onChange((Boolean build) -> {
            if (build) {
                buildAuto();
            }
        });
    }

    public void buildAuto() {
        Command[] autoCommands=new Command[GeneralConstants.numAuto];
        for(int i=0;i<GeneralConstants.numAuto;i++){
            autoCommands[i]=choices[i].getCommand();
        }
        auto = new PathPlannerAuto(new SequentialCommandGroup(autoCommands));
    }

    public PathPlannerAuto getAuto() {
        if (auto == null) {
            buildAuto();
        }
        return auto;
    }
}
