package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private SparkMax intakeMotor;

    public Intake(){
        intakeMotor = new SparkMax(25, MotorType.kBrushless);
    }

    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void stopIntakeMotor(){
        intakeMotor.stopMotor();
    }
}