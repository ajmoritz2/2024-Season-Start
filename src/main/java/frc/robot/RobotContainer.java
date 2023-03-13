// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auton.Autons;
import frc.robot.loops.SubsystemManager;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.SystemState;
import frc.robot.subsystems.Intake.WantedState;

import java.nio.file.Path;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	private static RobotContainer INSTANCE;

	public final XboxController driverController;
	public final PS4Controller operatorController;


	private final SubsystemManager manager;

	public final Drivetrain drivetrain;
	public final Arm arm;
	private final Intake intake;
	private final Limelight limelight;

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
		driverController = new XboxController(0);
		operatorController = new PS4Controller(1);
		// Configure the button bindings
		LiveWindow.disableAllTelemetry();
		LiveWindow.setEnabled(false);
		
		intake = new Intake(operatorController);
		arm = new Arm(operatorController,intake);
		drivetrain = new Drivetrain(driverController);
		limelight = new Limelight(driverController);
		
		manager = new SubsystemManager(0.02);

		manager.setSubsystems(drivetrain, arm, intake, limelight);


		addressButtons();
		configureButtonBindings();
		configureAuton();


	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */

	private void addressButtons(){
		
	 }
	private void configureButtonBindings() {
				
		//firstBall.and(secondBall).whenActive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.SPINUP)));

	}

	private void configureAuton() {
		autonChooser = new SendableChooser<>();
		//autonChooser.setDefaultOption("Do Nothing", new InstantCommand(() -> System.out.println("Doing nothing...")));
		autonChooser.addOption("center", Autons.center(drivetrain,arm,intake));
		autonChooser.addOption("clear", Autons.clear(drivetrain, arm, intake));
		autonChooser.addOption("wireCover", Autons.wireCover(drivetrain, arm, intake));
		autonChooser.addOption("EMERGENCY USE ONLY!!: NOTHING", Autons.emergencyDonNothing(drivetrain));
		SmartDashboard.putData("auton/chooser",autonChooser);

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