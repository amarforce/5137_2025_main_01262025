package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
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
import frc.robot.Constants.armConstants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Arm extends SubsystemBase{
    
    private TalonFX armMotor = new TalonFX(armConstants.motorId, "rhino");
    private MotorOutputConfigs motorOutput;
    private PIDController armPID;
    private double goal = 0;
    private final DutyCycleOut request = new DutyCycleOut(0.0);
    private ArmFeedforward feedFor;
    
    private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), armConstants.gearRatio, armConstants.jkg, armConstants.armLength, armConstants.min, armConstants.max, true, armConstants.min);

    public final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("arm") 
                        .voltage(Volts.of(armMotor.get() * RobotController.getBatteryVoltage()))
                        .angularPosition(Rotations.of(getPose()))
                        .angularVelocity(RotationsPerSecond.of(getVelocity()));
                },
                this));

    private final Mechanism2d mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Arm Root", 10, 0);
    private final MechanismLigament2d armMech2d = mech2dRoot.append(new MechanismLigament2d("Arm", 20, 90));
    
    private TalonFXSimState armMotorSim = armMotor.getSimState();
        
    public Arm() {
        
        motorOutput = new MotorOutputConfigs();
        armPID = new PIDController(armConstants.kP, armConstants.kI, armConstants.kP);
        feedFor = new ArmFeedforward(armConstants.ks, armConstants.kg, armConstants.kv);

   
        armPID.setTolerance(armConstants.tolerance);
        Shuffleboard.getTab("Arm Controller").add(armPID);
    }

    public void setArmUp() {
        motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotor.getConfigurator().apply(motorOutput);
    }
    public void setArmDown() {
        motorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armMotor.getConfigurator().apply(motorOutput);
    }

    public void setSpeed(double speed) {
        armMotor.setControl(request.withOutput(speed));
    }
    
    public double getPose() {
        return ((armMotor.getPosition().getValueAsDouble()+armConstants.armOffset)/armConstants.gearRatio);
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
        return armMotor.getVelocity().getValueAsDouble()/armConstants.gearRatio;
    }
    public void moveToLevel(double goal) {
        armPID.calculate(goal);
    } 
    public void telemetry() {
        SmartDashboard.putNumber("Arm pose", armSim.getAngleRads());
        SmartDashboard.putNumber("Arm Velocity", armSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("Arm Speed", armMotor.get());
        armMech2d.setAngle(getPose()*360);
        SmartDashboard.putData("Arm", mech2d);
    }

    @Override
    public void periodic(){
        telemetry();
        double extra = feedFor.calculate(goal, 0);
        double voltage = armPID.calculate(getPose(), goal)+extra;
        setVoltage(Volts.of(voltage));
    }

    @Override
    public void simulationPeriodic() {
        armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double armInput = armMotorSim.getMotorVoltage();
        armSim.setInputVoltage(armInput);
        armSim.update(0.02);
        double angle = armSim.getAngleRads()/(Math.PI*2);
        armMotorSim.setRotorVelocity(armSim.getVelocityRadPerSec()/(Math.PI*2)*armConstants.gearRatio);
        armMotorSim.setRawRotorPosition((angle*armConstants.gearRatio)-armConstants.armOffset);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    }


}

