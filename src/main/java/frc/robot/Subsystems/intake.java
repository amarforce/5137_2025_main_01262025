package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
    private SparkMax intakeMotor;

    public intake(){
        intakeMotor = new SparkMax(5, MotorType.kBrushless);
    }

    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void stopIntakeMotor(){
        intakeMotor.set(0.0);
        intakeMotor.stopMotor();
        
    }
}