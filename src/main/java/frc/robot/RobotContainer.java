// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.loops.SubsystemManager;
import frc.robot.subsystems.Arm;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	private static RobotContainer INSTANCE;

	public final XboxController driverController;
	public final XboxController operatorController;

	private final SubsystemManager manager;
	private final Arm arm;
	
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
		operatorController = new XboxController(1);
		// Configure the button bindings
		LiveWindow.disableAllTelemetry();
		LiveWindow.setEnabled(false);

		

		manager = new SubsystemManager(0.02);
		arm = new Arm(driverController);

		manager.setSubsystems(arm);

		configureAuton();


	}

	private void configureAuton() {
		

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
