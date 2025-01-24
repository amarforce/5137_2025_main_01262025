package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
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

import static edu.wpi.first.units.Units.Rotations;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Arm extends SubsystemBase{
    
    private TalonFX armMotor = new TalonFX(ArmConstants.motorId, "rhino");
    private PIDController armController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kP);
    private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    private double goal = 0;
    
    private SingleJointedArmSim armSim = new SingleJointedArmSim(ArmConstants.motorSim, ArmConstants.gearRatio, ArmConstants.jkg, ArmConstants.armLength, ArmConstants.min, ArmConstants.max, true, ArmConstants.min);

    public final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("arm")
                        .voltage(Volts.of(armMotor.get() * RobotController.getBatteryVoltage()))
                        .angularPosition(Rotations.of(getMeasurement()))
                        .angularVelocity(RotationsPerSecond.of(getVelocity()));
                },
                this));

    private final Mechanism2d mech2d = new Mechanism2d(ArmConstants.mechWidth, ArmConstants.mechHeight);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Arm Root", ArmConstants.mechWidth/2, ArmConstants.mechHeight/2);
    private final MechanismLigament2d armMech2d = mech2dRoot.append(new MechanismLigament2d("Arm", 20, 90));
    
    private TalonFXSimState armMotorSim = armMotor.getSimState();
        
    public Arm() {
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        armMotor.getConfigurator().apply(currentConfigs);

        armController.setTolerance(ArmConstants.tolerance);
        SmartDashboard.putData("Arm Controller",armController);
    }

    public void setSpeed(double speed) {
        armMotor.set(speed);
    }
    
    public double getMeasurement() {
        return armMotor.getPosition().getValueAsDouble()/ArmConstants.gearRatio-ArmConstants.armOffset;
    }
    
    public void setGoal(double newGoal){
        goal = newGoal;
    }

    public double getGoal(){
        return goal;
    }

    public void setVoltage(Voltage v) {
        armMotor.setVoltage(v.magnitude());
    }

    public double getVelocity() {
        return armMotor.getVelocity().getValueAsDouble()/ArmConstants.gearRatio;
    }

    public void telemetry() {
        SmartDashboard.putNumber("Arm Angle", armSim.getAngleRads());
        SmartDashboard.putNumber("Arm Velocity", armSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("Arm Input", armMotor.get());
        armMech2d.setAngle(getMeasurement()*360);
        SmartDashboard.putData("Arm", mech2d);
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
        double angle = armSim.getAngleRads()/(2*Math.PI);
        double vel = armSim.getVelocityRadPerSec()/(2*Math.PI);
        armMotorSim.setRotorVelocity(vel*ArmConstants.gearRatio);
        armMotorSim.setRawRotorPosition((angle+ArmConstants.armOffset)*ArmConstants.gearRatio);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }
}