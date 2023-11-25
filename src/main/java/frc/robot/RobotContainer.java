package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.loops.SubsystemManager;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    private static RobotContainer INSTANCE;

    private final SubsystemManager manager;

	public final Drivetrain drivetrain;

    public XboxController driveController;

	private SendableChooser<Command> autonChooser;

	public enum EnableState {
		DISABLED,
		AUTON,
		TELEOP
	}

	public EnableState enableState = EnableState.DISABLED;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		INSTANCE = this;
		driveController = new XboxController(Constants.Controllers.driveID);
		// Configure the button bindings
		LiveWindow.disableAllTelemetry();
		LiveWindow.setEnabled(false);
		
		drivetrain = new Drivetrain(driveController);

		manager = new SubsystemManager(Constants.loopLengthSeconds);

		manager.setSubsystems(drivetrain);
	}

	

	public static synchronized RobotContainer getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new RobotContainer();
		}
		return INSTANCE;
	}

	public SendableChooser<Command> getAutonChooser() {
		return autonChooser;
	}

	public void startSubsystemThreads(){
		manager.start();
	}

	public void stopSubsystemThreads(){
		manager.stop();
	}

	public void checkSubsystems() {
		manager.checkSubsystems();
	}

	public void stopSubsystems() {
		manager.stopSubsystems();
	}
    
}
