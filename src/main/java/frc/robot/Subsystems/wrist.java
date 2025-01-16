package frc.robot.Subsystems;



import edu.wpi.first.units.measure.Angle;

//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.Wrist_Constants;
import edu.wpi.first.math.util.Units;

//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class wrist extends SubsystemBase{
    private TalonFX wristMotor;

    public wrist(){
        wristMotor = new TalonFX(4, "rhino");
        
    }
    public void setWristSpeed(double speed){
        wristMotor.set(speed);
    }
    
    public void stopWristMotor(){
        wristMotor.set(0.0);
        wristMotor.setNeutralMode(NeutralModeValue.Brake);
        
    }

    public void wristPos1(){
        wristMotor.setPosition(90);
    }
    public void wristPos2(){
        wristMotor.setPosition(0);
    }
    public double wristPosition(){
        return wristMotor.getPosition().getValueAsDouble();
    } 
}




