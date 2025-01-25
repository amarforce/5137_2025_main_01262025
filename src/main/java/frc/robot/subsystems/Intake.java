package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private SparkMax intakeMotor;

    public Intake(){
        intakeMotor = new SparkMax(IntakeConstants.motorId, MotorType.kBrushless);
    }

    public void setSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void stop(){
        intakeMotor.stopMotor();
    }
}