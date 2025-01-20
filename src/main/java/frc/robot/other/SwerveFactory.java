package frc.robot.other;

import frc.robot.constants.SwerveConstants;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

public class SwerveFactory {

    private static JSONParser parser = new JSONParser();

    private static double maxSpeed;

    public static SwerveDrivetrain<TalonFX, TalonFX, CANcoder> createSwerve(File file) {
        try {     
            JSONObject constants =  (JSONObject) parser.parse(new FileReader(file));

            JSONObject drive_gains = (JSONObject) constants.get("drive_gains");
            JSONObject steer_gains = (JSONObject) constants.get("steer_gains");

            maxSpeed = (double) constants.get("max_speed");

            SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
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
                            .withStatorCurrentLimit(Amps.of(60))
                            .withStatorCurrentLimitEnable(true)))
                    .withEncoderInitialConfigs(new CANcoderConfiguration())
                    .withSteerInertia(KilogramSquareMeters.of(0.01)) // Only used for simulation
                    .withDriveInertia(KilogramSquareMeters.of(0.01)) // Only used for simulation
                    .withSteerFrictionVoltage(Volts.of(0.2)) // Only used for simulation
                    .withDriveFrictionVoltage(Volts.of(0.2)); // Only used for simulation
            
            JSONObject modules = (JSONObject) constants.get("modules");

            JSONObject front_left = (JSONObject) modules.get("front_left");
            JSONObject front_right = (JSONObject) modules.get("front_right");
            JSONObject back_left = (JSONObject) modules.get("back_left");
            JSONObject back_right = (JSONObject) modules.get("back_right");

            // Integers cannot be cast directly from Object, and must be cast to long before being cast to int

            return new SwerveDrivetrain<TalonFX, TalonFX, CANcoder> (

                TalonFX::new, TalonFX::new, CANcoder::new,

                new SwerveDrivetrainConstants()
                    .withCANBusName("rio")
                    .withPigeon2Id((int) (long) constants.get("gyro_id"))
                    .withPigeon2Configs(null),
                
                SwerveConstants.odometryFrequency,

                ConstantCreator.createModuleConstants(
                    (int) (long) ((JSONObject) front_left.get("angle_motor")).get("id"),
                    (int) (long) ((JSONObject) front_left.get("drive_motor")).get("id"),
                    (int) (long) ((JSONObject) front_left.get("encoder")).get("id"),
                    Rotations.of((double) ((JSONObject) front_left.get("encoder")).get("offset")),
                    Inches.of((double) (long) front_left.get("x")),
                    Inches.of((double) (long) front_left.get("y")),
                    (boolean) ((JSONObject) front_left.get("drive_motor")).get("inverted"),
                    (boolean) ((JSONObject) front_left.get("angle_motor")).get("inverted"),
                    (boolean) ((JSONObject) front_left.get("encoder")).get("inverted")
                ),

                ConstantCreator.createModuleConstants(
                    (int) (long) ((JSONObject) front_right.get("angle_motor")).get("id"),
                    (int) (long) ((JSONObject) front_right.get("drive_motor")).get("id"),
                    (int) (long) ((JSONObject) front_right.get("encoder")).get("id"),
                    Rotations.of((double) ((JSONObject) front_right.get("encoder")).get("offset")),
                    Inches.of((double) (long) front_right.get("x")),
                    Inches.of((double) (long) front_right.get("y")),
                    (boolean) ((JSONObject) front_right.get("drive_motor")).get("inverted"),
                    (boolean) ((JSONObject) front_right.get("angle_motor")).get("inverted"),
                    (boolean) ((JSONObject) front_right.get("encoder")).get("inverted")
                ),

                ConstantCreator.createModuleConstants(
                    (int) (long) ((JSONObject) back_left.get("angle_motor")).get("id"),
                    (int) (long) ((JSONObject) back_left.get("drive_motor")).get("id"),
                    (int) (long) ((JSONObject) back_left.get("encoder")).get("id"),
                    Rotations.of((double) ((JSONObject) back_left.get("encoder")).get("offset")),
                    Inches.of((double) (long) back_left.get("x")),
                    Inches.of((double) (long) back_left.get("y")),
                    (boolean) ((JSONObject) back_left.get("drive_motor")).get("inverted"),
                    (boolean) ((JSONObject) back_left.get("angle_motor")).get("inverted"),
                    (boolean) ((JSONObject) back_left.get("encoder")).get("inverted")
                ),

                ConstantCreator.createModuleConstants(
                    (int) (long) ((JSONObject) back_right.get("angle_motor")).get("id"),
                    (int) (long) ((JSONObject) back_right.get("drive_motor")).get("id"),
                    (int) (long) ((JSONObject) back_right.get("encoder")).get("id"),
                    Rotations.of((double) ((JSONObject) back_right.get("encoder")).get("offset")),
                    Inches.of((double) (long) back_right.get("x")),
                    Inches.of((double) (long) back_right.get("y")),
                    (boolean) ((JSONObject) back_right.get("drive_motor")).get("inverted"),
                    (boolean) ((JSONObject) back_right.get("angle_motor")).get("inverted"),
                    (boolean) ((JSONObject) back_right.get("encoder")).get("inverted")
                )
            );
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public static double getMaxSpeed() {
        return maxSpeed;
    }
}