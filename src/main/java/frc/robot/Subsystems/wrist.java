package frc.robot.Subsystems;

//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.Wrist_Constants;


//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class wrist extends SubsystemBase{
    private TalonFX wristMotor;
    private TalonFX intakeMotor;

    public wrist(){
        wristMotor = new TalonFX(4, "rhino");
        intakeMotor = new TalonFX(5, "rhino");
    }
    public void setWristSpeed(double speed){
        wristMotor.set(speed);
    }
    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }
    public void stopWristMotor(){
        wristMotor.set(0.0);
        wristMotor.setNeutralMode(NeutralModeValue.Brake);
        
    }
    public void stopIntakeMotor(){
        intakeMotor.set(0.0);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        
    }

}

