package frc.robot.Subsystems;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist_Constants;
import edu.wpi.first.math.system.plant.DCMotor;
//import frc.robot.Constants.Wrist_Constants;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
//simulation stuff
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class wrist extends SubsystemBase{
    private TalonFX wristMotor = new TalonFX(4, "rhino");
    private TalonFXSimState fakeWristMotor = wristMotor.getSimState();

    private final Mechanism2d mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("WristRoot", 10, 0);
    private final MechanismLigament2d armMech2d = mech2dRoot.append(new MechanismLigament2d("Wrist", 20, 90));

    private SingleJointedArmSim wristSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), Wrist_Constants.gearRatio, Wrist_Constants.jkg, Wrist_Constants.wristLength, Wrist_Constants.min, Wrist_Constants.max, true, Wrist_Constants.min);

    public void setWristSpeed(double speed){
        wristMotor.set(speed);
    }
    
    public void stopWristMotor(){
        wristMotor.set(0.0);
        wristMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void wristPos1(){
        wristMotor.setPosition(1);
    }

    public void wristPos2(){
        wristMotor.setPosition(0.25);
    }

    public double wristPosition(){
        
    
        return wristMotor.getPosition().getValueAsDouble();
    }


    public double getPose() {
        return ((wristMotor.getPosition().getValueAsDouble()+Wrist_Constants.wristOffset)/Wrist_Constants.gearRatio);
    }




    public void telemetry() {
        SmartDashboard.putNumber("Wrist pose", wristSim.getAngleRads());
        SmartDashboard.putNumber("Wrist Velocity", wristSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("Wrist Speed", wristMotor.get());
        armMech2d.setAngle(getPose()*360);
        SmartDashboard.putData("Wrist", mech2d);
    }

    @Override
    public void periodic(){
        telemetry();
    }










    @Override
    public void simulationPeriodic(){
        fakeWristMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
        double wristInput = fakeWristMotor.getMotorVoltage();
        wristSim.setInputVoltage(wristInput);
        wristSim.update(0.02);
        double angle = wristSim.getAngleRads()/(Math.PI*2);

        fakeWristMotor.setRotorVelocity(wristSim.getVelocityRadPerSec()/(Math.PI*2)*Wrist_Constants.gearRatio);
        fakeWristMotor.setRawRotorPosition((angle*Wrist_Constants.gearRatio)-Wrist_Constants.wristOffset);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wristSim.getCurrentDrawAmps()));
    }
}





