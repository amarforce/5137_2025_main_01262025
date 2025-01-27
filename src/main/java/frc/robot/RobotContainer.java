package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// Add other necessary imports here

import frc.robot.elastic.*;
import frc.robot.other.ArmMechanism;
import frc.robot.other.AutoFactory;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.recipes.coral.DropCoralL4Command;

@SuppressWarnings("unused")
public class RobotContainer {
	private CommandPS5Controller driver;
	private CommandPS5Controller operator;

	private Vision vision;
	private Swerve swerve;
	private Elevator elevator;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
  	private Hang hang;
	private ArmMechanism armMechanism;

	private SwerveCommands swerveCommands;
	private ElevatorCommands elevatorCommands;
	private ArmCommands armCommands;
	private WristCommands wristCommands;
	private IntakeCommands intakeCommands;
  	private HangCommand hangCommand;
	private MultiCommands multiCommands;

	private Reef reef;
	private ReefScoring reefScoring;

	private AutoFactory autoFactory;

	public RobotContainer() {
		driver = new CommandPS5Controller(0);
		operator = new CommandPS5Controller(1);

		reef = new Reef();
		reefScoring = new ReefScoring(reef);
		SmartDashboard.putData("Reef", reef);
		SmartDashboard.putData("ReefScoring", reefScoring);

		vision = new Vision(reef);
		swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve.json"), vision);
		elevator = new Elevator();
		arm = new Arm();
		wrist = new Wrist();
		intake = new Intake();
    	hang = new Hang();
		armMechanism = new ArmMechanism(arm, elevator, wrist);

		swerveCommands = new SwerveCommands(swerve);
		elevatorCommands = new ElevatorCommands(elevator);
		armCommands = new ArmCommands(arm);
		wristCommands = new WristCommands(wrist);
		intakeCommands = new IntakeCommands(intake);
    	hangCommand = new HangCommand(hang);
		multiCommands = new MultiCommands(arm,elevator,wrist,swerve,intake,hang,armCommands,elevatorCommands,wristCommands,swerveCommands,intakeCommands,hangCommand);

        // Create the L4 placement command
        Command dropCoralL4 = new DropCoralL4Command(
            multiCommands,
            swerveCommands,
            intakeCommands
        );

		// Add to SmartDashboard for testing
		SmartDashboard.putData("Drop Coral L4", dropCoralL4);
		
		configureBindings();

		autoFactory = new AutoFactory(multiCommands);
	}

	private void configureBindings() {
		// Driver Bindings

		swerve.setDefaultCommand(
			swerveCommands.drive(
				() -> -driver.getLeftY(),
				() -> -driver.getLeftX(),
				() -> -driver.getRightX(),
				() -> driver.R1().negate().getAsBoolean())
		);

		driver.cross().whileTrue(swerveCommands.lock());

		driver.triangle().onTrue(swerveCommands.driveToStation());
		driver.square().onTrue(swerveCommands.driveToCage());
		driver.circle().onTrue(swerveCommands.driveToProcessor());

		driver.povLeft().onTrue(swerveCommands.driveToReefLeft());
		driver.povUp().onTrue(swerveCommands.driveToReefCenter());
		driver.povRight().onTrue(swerveCommands.driveToReefRight());

		driver.options().onTrue(swerveCommands.resetGyro());

		// Cancel all commands
		driver.touchpad().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		// For testing
		elevator.setDefaultCommand(elevatorCommands.changeGoal(()->-operator.getLeftY()/50));

		arm.setDefaultCommand(armCommands.changeGoal(() ->-operator.getLeftX()/50));

		// Operator Bindings

		operator.triangle()
			.onTrue(multiCommands.moveToGoal(4));

		operator.circle()
			.onTrue(multiCommands.moveToGoal(3));

		operator.square()
			.onTrue(multiCommands.moveToGoal(2));

		operator.cross()
			.onTrue(multiCommands.moveToGoal(1));

		operator.R1()
			.onTrue(wristCommands.toPos1())
			.onFalse(wristCommands.toPos2());

		operator.L2()
			.onTrue(intakeCommands.outtake())
			.onFalse(intakeCommands.stop());

		operator.R2()
			.onTrue(intakeCommands.intakeUntilSwitched())
			.onFalse(intakeCommands.stop());
    
		operator.touchpad()
		.onTrue(hangCommand);


        // Create the L4 placement command
        Command dropCoralL4 = new DropCoralL4Command(
            multiCommands,
            swerveCommands,
            intakeCommands
        );

	// Bind to operator controls (example)
		operator.L1().onTrue(dropCoralL4);
	}
    

    public Command getAutonomousCommand() {
		return autoFactory.getAuto();
	}
}