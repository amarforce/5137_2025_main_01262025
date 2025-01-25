package frc.robot.other;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.MechanismConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ArmMechanism extends SubsystemBase{
    private Arm arm;
    private Elevator elevator;

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d mech2d = new Mechanism2d(MechanismConstants.mechWidth, MechanismConstants.mechHeight);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", MechanismConstants.mechWidth / 2, 0);
    private final MechanismLigament2d elevatorMech2d = mech2dRoot.append(new MechanismLigament2d("Elevator", 0, 90));
    private final MechanismLigament2d armMech2d = elevatorMech2d.append(new MechanismLigament2d("Arm", MechanismConstants.armLength, 0));

    public ArmMechanism(Arm arm,Elevator elevator){
        this.arm=arm;
        this.elevator=elevator;
        elevatorMech2d.setColor(MechanismConstants.elevatorColor);
        armMech2d.setColor(MechanismConstants.armColor);
        SmartDashboard.putData("Arm",mech2d);
    }

    @Override
    public void periodic(){
        elevatorMech2d.setLength((elevator.getMeasurement() / (ElevatorConstants.maxHeight*2) + 0.25)*MechanismConstants.mechHeight);
        armMech2d.setAngle(Units.radiansToDegrees(arm.getMeasurement()-Math.PI/2));
    }
}
