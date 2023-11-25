package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.RobotController;

public class Robot extends LoggedRobot {

    public static RobotContainer robotContainer;

    @Override
    public void robotInit(){
        robotContainer = new RobotContainer(); // MUST CALL FIRST. INITIALIZES CONTROLLERS
        robotContainer.startSubsystemThreads();
    }

    @Override
    public void autonomousInit(){

    }


    @Override
    public void teleopInit(){
        robotContainer.enableState = RobotContainer.EnableState.TELEOP;
    }

    @Override
    public void simulationInit(){
        robotContainer.enableState = RobotContainer.EnableState.TELEOP;
    }

    @Override
    public void disabledInit(){
        robotContainer.stopSubsystems();
    }

    @Override
    public void autonomousPeriodic(){

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        robotContainer.drivetrain.getRawDrivetrain().updateSimState(0.020, RobotController.getBatteryVoltage());
    }

    @Override
    public void disabledPeriodic(){

    }

    @Override
    public void autonomousExit(){

    }

    @Override
    public void teleopExit(){

    }

    @Override
    public void disabledExit(){
        robotContainer.stopSubsystemThreads();
    }
}
