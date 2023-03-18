// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;


public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static CTREConfigs ctreConfigs;

  private Command autonomousCommand;

  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.startSubsystemThreads();

    for (String key : SmartDashboard.getKeys()){ // Get rid of unused/ commented out keys hopefully.
      SmartDashboard.clearPersistent(key);
    }

    m_robotContainer.arm.zeroArmSensors();
    m_robotContainer.arm.zeroRotateSensors();
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
    m_robotContainer.drivetrain.resetOdometry();

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
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void disabledInit() {
    m_robotContainer.enableState = RobotContainer.EnableState.DISABLED;
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
