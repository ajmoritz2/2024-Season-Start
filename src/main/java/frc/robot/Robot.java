// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;


public class Robot extends LoggedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static CTREConfigs ctreConfigs;

  private Command autonomousCommand;

  public static RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    Logger.getInstance().recordMetadata("Argentina", "Main Branch"); // Set a metadata value

    if (isReal()) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

// Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.


    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.startSubsystemThreads();

    for (String key : SmartDashboard.getKeys()){ // Get rid of unused/ commented out keys hopefully.
      SmartDashboard.clearPersistent(key);
    }

    m_robotContainer.arm.zeroExtendSensor();
    m_robotContainer.arm.zeroRotateSensor();
    m_robotContainer.limelight.setWantedState(Limelight.SystemState.NEUTRAL);
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
  }


  @Override
  public void autonomousInit() {
    m_robotContainer.enableState = RobotContainer.EnableState.AUTON;
    m_robotContainer.drivetrain.getBaseDrivetrain().resetOdometry();

		autonomousCommand = m_robotContainer.getAutonChooser().getSelected();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    m_robotContainer.enableState = RobotContainer.EnableState.TELEOP;
    m_robotContainer.drivetrain.setWantedState(Drivetrain.WantedState.MANUAL_CONTROL);
    m_robotContainer.limelight.setWantedState(Limelight.SystemState.NEUTRAL);
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void disabledInit() {
    // TODO: Test to see if these break/make the robot better
    m_robotContainer.enableState = RobotContainer.EnableState.DISABLED;
    m_robotContainer.drivetrain.setWantedState(Drivetrain.WantedState.IDLE);
    m_robotContainer.stopSubsystems();

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}