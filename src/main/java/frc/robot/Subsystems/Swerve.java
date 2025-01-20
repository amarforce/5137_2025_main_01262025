package frc.robot.subsystems;

import frc.robot.other.SwerveFactory;
import frc.robot.other.Telemetry;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {

    private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve;

    private Vision vision;

    private double maxSpeed;
    private double maxAngularSpeed;

    private Telemetry logger;

    private SwerveRequest.FieldCentric fieldOrientedDrive;
    private SwerveRequest.RobotCentric robotOrientedDrive;

    public Swerve(File file, Vision vision) {
        swerve = SwerveFactory.createSwerve(file);
        this.vision = vision;

        maxSpeed = SwerveFactory.getMaxSpeed();
        maxAngularSpeed = 1.5*Math.PI;

        DriverStation.getAlliance().ifPresent(allianceColor -> {
                swerve.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero
                );
            });
                
        fieldOrientedDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        robotOrientedDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        if (RobotBase.isSimulation()) {
            startSimThread();
        }

        logger = new Telemetry(maxSpeed);

        swerve.registerTelemetry(logger::telemeterize);
    }

    public void setControl(SwerveRequest request) {
        swerve.setControl(request);
    }

    public void percentOutput(double percentX, double percentY, double percentTheta, boolean fieldRelative) {
        if (fieldRelative) {
            setControl(fieldOrientedDrive
                .withVelocityX(percentX*maxSpeed)
                .withVelocityY(percentY*maxSpeed)
                .withRotationalRate(percentTheta*maxAngularSpeed)
            );
        } else {
            setControl(robotOrientedDrive
                .withVelocityX(percentX*maxSpeed)
                .withVelocityY(percentY*maxSpeed)
                .withRotationalRate(percentTheta*maxAngularSpeed)
            );
        }
    }

    public void reset() {
        DriverStation.getAlliance().ifPresent(allianceColor -> {
            swerve.setOperatorPerspectiveForward(
                allianceColor == Alliance.Red
                    ? Rotation2d.k180deg
                    : Rotation2d.kZero
            );
        });
        swerve.seedFieldCentric();
    }

    public void lock() {
        setControl(new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d(0, 0)));
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> frontPose = vision.getFrontPoseEstimate();
        Optional<EstimatedRobotPose> leftPose = vision.getFrontPoseEstimate();

        if (frontPose.isPresent()) {
            swerve.addVisionMeasurement(
            frontPose.get().estimatedPose.toPose2d(),
            frontPose.get().timestampSeconds);
        }
        
        if (leftPose.isPresent()) {
            swerve.addVisionMeasurement(
            leftPose.get().estimatedPose.toPose2d(),
            leftPose.get().timestampSeconds);
        }
    }

    // CTRE Generated SysId Routines

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    public final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(new SwerveRequest.SysIdSwerveTranslation().withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    public final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(new SwerveRequest.SysIdSwerveSteerGains().withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    public final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(new SwerveRequest.SysIdSwerveRotation().withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine routine = m_sysIdRoutineTranslation;

    public void setRoutine(SysIdRoutine routine) {
        this.routine = routine;
    }

    public SysIdRoutine getRoutine() {
        return routine;
    }

    // CTRE Telemtry Simulation

    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            swerve.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        swerve.registerTelemetry(telemetryFunction);
    }
}
