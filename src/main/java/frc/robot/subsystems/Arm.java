package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ArmConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import static edu.wpi.first.units.Units.Volts;

public class Arm extends SubsystemBase{
    
    private TalonFX armMotor = new TalonFX(ArmConstants.motorId, "rhino");
    private PIDController armController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    private double goal = ArmConstants.defaultGoal;
    
    private SingleJointedArmSim armSim = new SingleJointedArmSim(ArmConstants.motorSim, ArmConstants.gearRatio, ArmConstants.momentOfInertia, ArmConstants.armLength, ArmConstants.minAngle, ArmConstants.maxAngle, true, ArmConstants.defaultGoal);

    public final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("arm")
                        .voltage(Volts.of(armMotor.get() * RobotController.getBatteryVoltage()))
                        .angularPosition(Radians.of(getMeasurement()))
                        .angularVelocity(RadiansPerSecond.of(getVelocity()));
                },
                this));
    
    private TalonFXSimState armMotorSim = armMotor.getSimState();
        
    public Arm() {
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        armMotor.getConfigurator().apply(currentConfigs);

        armController.setTolerance(ArmConstants.armTolerance);
        SmartDashboard.putData("Arm Controller",armController);
    }

    public void setSpeed(double speed) {
        armMotor.set(speed);
    }
    
    // Returns measurement in radians
    public double getMeasurement() {
        return Units.rotationsToRadians(armMotor.getPosition().getValueAsDouble()/ArmConstants.gearRatio)-ArmConstants.armOffset;
    }
    
    public void setGoal(double newGoal){
        if(newGoal<ArmConstants.minAngle){
            newGoal=ArmConstants.minAngle;
        }
        if(newGoal>ArmConstants.maxAngle){
            newGoal=ArmConstants.maxAngle;
        }
        goal = newGoal;
    }

    public double getGoal(){
        return goal;
    }

    public void setVoltage(Voltage v) {
        armMotor.setVoltage(v.magnitude());
    }

    // Returns velocity in radians/sec
    public double getVelocity() {
        return Units.rotationsToRadians(armMotor.getVelocity().getValueAsDouble()/ArmConstants.gearRatio);
    }

    public void telemetry() {
        SmartDashboard.putNumber("Arm Angle", armSim.getAngleRads());
        SmartDashboard.putNumber("Arm Velocity", armSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("Arm Input", armMotor.get());
    }

    @Override
    public void periodic(){
        telemetry();
        double extra = feedforward.calculate(getMeasurement(), getVelocity());
        double voltage = armController.calculate(getMeasurement(), goal)+extra;
        setVoltage(Volts.of(voltage));
    }

    @Override
    public void simulationPeriodic() {
        armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double armInput = armMotorSim.getMotorVoltage();
        armSim.setInputVoltage(armInput);
        armSim.update(ArmConstants.simPeriod);
        double angle = armSim.getAngleRads();
        armMotorSim.setRawRotorPosition(Units.radiansToRotations((angle+ArmConstants.armOffset)*ArmConstants.gearRatio));
        double vel = armSim.getVelocityRadPerSec();
        armMotorSim.setRotorVelocity(Units.radiansToRotations(vel*ArmConstants.gearRatio));
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }
}