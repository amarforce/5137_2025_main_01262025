package frc.robot.subsystems;

import frc.robot.constants.SwerveConstants;
import frc.robot.other.SwerveFactory;
import frc.robot.other.Telemetry;

import static edu.wpi.first.units.Units.*;

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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {

    private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve;

    private Vision vision;

    private double maxSpeed;
    private double maxAngularSpeed;

    private Telemetry logger;
    private Field2d field;

    private SwerveRequest.FieldCentric fieldOrientedDrive;
    private SwerveRequest.RobotCentric robotOrientedDrive;

    private SendableChooser<Integer> cageChoice;

    public Swerve(File file, Vision vision) {
        swerve = SwerveFactory.createSwerve(file);
        this.vision = vision;

        maxSpeed = SwerveFactory.getMaxSpeed();
        maxAngularSpeed = 1.5*Math.PI;
                
        fieldOrientedDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * SwerveConstants.translationalDeadband).withRotationalDeadband(maxAngularSpeed * SwerveConstants.rotationalDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        robotOrientedDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * SwerveConstants.translationalDeadband).withRotationalDeadband(maxAngularSpeed * SwerveConstants.rotationalDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        RobotConfig config;
        try {
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getCurrentSpeeds,
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(SwerveConstants.translation_kP, SwerveConstants.translation_kI, SwerveConstants.translation_kD),
                        new PIDConstants(SwerveConstants.rotation_kP, SwerveConstants.rotation_kI, SwerveConstants.rotation_kD)
                ),
                config,
                () -> onRedAlliance(),
                this
        );

        cageChoice = new SendableChooser<Integer>();
        cageChoice.addOption("Left", 0);
        cageChoice.addOption("Center", 1);
        cageChoice.addOption("Right", 2);
        cageChoice.setDefaultOption("Center", 1);
        SmartDashboard.putData("Cage Choice", cageChoice);
        
        if (RobotBase.isSimulation()) {
            startSimThread();
        }

        // Might remove telemtry, keep for now
        logger = new Telemetry(maxSpeed);
        swerve.registerTelemetry(logger::telemeterize);

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void setControl(SwerveRequest request) {
        swerve.setControl(request);
    }

    public void resetPose(Pose2d pose) {
        swerve.resetPose(pose);
    }

    public Pose2d getPose() {
        return swerve.getState().Pose;
    }

    public void drive(ChassisSpeeds speeds) {
        setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return swerve.getState().Speeds;
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

    public void driveToPose(Pose2d pose) {
        if (onRedAlliance()) {
            pose = invertPose(pose);
        }
        Command path = AutoBuilder.pathfindToPose(pose, new PathConstraints(
            MetersPerSecond.of(5.0),
            MetersPerSecondPerSecond.of(5.0),
            RadiansPerSecond.of(1.5*Math.PI),
            RadiansPerSecondPerSecond.of(Math.PI)),
            MetersPerSecond.of(0.0));
        path.addRequirements(this);
        path.schedule();
    }

    public Pose2d getClosest(Pose2d[] poses) {
        Pose2d pose = this.getPose();
        if (this.onRedAlliance()) {
            pose = this.invertPose(pose);
        }
        int closest = 0;
        double closest_distance = 100.0;
        for (int i = 0; i < poses.length; i++) {
            Transform2d transform = pose.minus(poses[i]);
            double distance = Math.hypot(transform.getX(), transform.getY());
            if (distance < closest_distance) {
                closest_distance = distance;
                closest = i;
            }
        }
        return poses[closest];
    }

    public Pose2d invertPose(Pose2d pose) {
        return new Pose2d(SwerveConstants.fieldLength - pose.getX(), SwerveConstants.fieldWidth - pose.getY(), pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    public boolean onRedAlliance() {
        var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
        return false;
    }

    public int getCage() {
        return cageChoice.getSelected();
    }

    public void resetGyro() {
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
        //TODO: Add this
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

        field.setRobotPose(this.getPose());
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
