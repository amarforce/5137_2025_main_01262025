package frc.robot.other;

import frc.robot.constants.SwerveConstants;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.io.FileReader;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

public class SwerveFactory {

    private JSONParser parser = new JSONParser();

    private double maxSpeed;
    private double maxAngularSpeed;
    private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

    public SwerveFactory(File file){
        try {     
            JSONObject constants =  (JSONObject) parser.parse(new FileReader(file));

            JSONObject drive_gains = (JSONObject) constants.get("drive_gains");
            JSONObject steer_gains = (JSONObject) constants.get("steer_gains");

            maxSpeed = (double) constants.get("max_speed");
            maxAngularSpeed = (double) constants.get("max_angular_speed");

            SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constantCreator =
                new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio((double) constants.get("drive_ratio"))
                    .withSteerMotorGearRatio((double) constants.get("steer_ratio"))
                    .withCouplingGearRatio((double) constants.get("couple_ratio"))
                    .withWheelRadius(Inches.of((double) constants.get("wheel_radius")))
                    .withDriveMotorGains(new Slot0Configs()
                        .withKP((double) drive_gains.get("P"))
                        .withKI((double) drive_gains.get("I"))
                        .withKD((double) drive_gains.get("D"))
                        .withKS((double) drive_gains.get("kS"))
                        .withKV((double) drive_gains.get("kV")))
                    .withSteerMotorGains(new Slot0Configs()
                        .withKP((double) steer_gains.get("P"))
                        .withKI((double) steer_gains.get("I"))
                        .withKD((double) steer_gains.get("D"))
                        .withKS((double) steer_gains.get("kS"))
                        .withKV((double) steer_gains.get("kV"))
                        .withKA((double) steer_gains.get("kA"))
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withSlipCurrent(Amps.of((double) constants.get("slip_current")))
                    .withSpeedAt12Volts(MetersPerSecond.of((double) constants.get("max_speed")))
                    .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                    .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                    .withSteerMotorInitialConfigs(new TalonFXConfiguration()
                        .withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(Amps.of((double) constants.get("current_limit")))
                            .withStatorCurrentLimitEnable(true)))
                    .withEncoderInitialConfigs(new CANcoderConfiguration())
                    .withSteerInertia(KilogramSquareMeters.of((double) constants.get("steer_inertia"))) // Only used for simulation
                    .withDriveInertia(KilogramSquareMeters.of((double) constants.get("drive_inertia"))) // Only used for simulation
                    .withSteerFrictionVoltage(Volts.of((double) constants.get("steer_friction_voltage"))) // Only used for simulation
                    .withDriveFrictionVoltage(Volts.of((double) constants.get("drive_friction_voltage"))); // Only used for simulation
            
            JSONObject modules = (JSONObject) constants.get("modules");

            JSONObject frontLeft = (JSONObject) modules.get("front_left");
            JSONObject frontRight = (JSONObject) modules.get("front_right");
            JSONObject backLeft = (JSONObject) modules.get("back_left");
            JSONObject backRight = (JSONObject) modules.get("back_right");

            // Integers cannot be cast directly from Long, and must be cast to long before being cast to int

            drivetrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder> (

                TalonFX::new, TalonFX::new, CANcoder::new,

                new SwerveDrivetrainConstants()
                    .withCANBusName("rio")
                    .withPigeon2Id((int) (long) constants.get("gyro_id"))
                    .withPigeon2Configs(null),
                
                SwerveConstants.odometryFrequency,

                getConstants(frontLeft, constantCreator),
                getConstants(frontRight, constantCreator),
                getConstants(backLeft, constantCreator),
                getConstants(backRight, constantCreator)
            );
        } catch (Exception e){
            throw new RuntimeException(e);
        }
    }

    private SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> getConstants(JSONObject module,SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constantCreator){
        return constantCreator.createModuleConstants(
            (int) (long) ((JSONObject) module.get("angle_motor")).get("id"),
            (int) (long) ((JSONObject) module.get("drive_motor")).get("id"),
            (int) (long) ((JSONObject) module.get("encoder")).get("id"),
            Rotations.of((double) ((JSONObject) module.get("encoder")).get("offset")),
            Inches.of((double) (long) module.get("x")),
            Inches.of((double) (long) module.get("y")),
            (boolean) ((JSONObject) module.get("drive_motor")).get("inverted"),
            (boolean) ((JSONObject) module.get("angle_motor")).get("inverted"),
            (boolean) ((JSONObject) module.get("encoder")).get("inverted")
        );
    }

    public SwerveDrivetrain<TalonFX, TalonFX, CANcoder> create() {
        return drivetrain;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxAngularSpeed() {
        return maxAngularSpeed;
    }
}