package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Drivetrain implements Subsystem {

    public static final double[] XY_Axis_inputBreakpoints = {-1, -0.85, -0.6, -0.12, 0.12, 0.6, 0.85, 1};
    public static final double[] XY_Axis_outputTable = {-1.0, -0.6, -0.3, 0, 0, 0.3, 0.6, 1.0};
    public static final double[] RotAxis_inputBreakpoints = {-1, -0.9, -0.6, -0.12, 0.12, 0.6, 0.9, 1};
    public static final double[] RotAxis_outputTable = {-1.0, -0.5, -0.2, 0, 0, 0.2, 0.5, 1.0};

    private enum WantedState {
        IDLE,
        MANUAL
    }

    private enum SystemState {
        IDLE,
        MANUAL
    }

    private static class PeriodicIO {
        // INPUTS
        @SuppressWarnings("unused")
        double timestamp;

        double modifiedJoystickX;
        double modifiedJoystickY;
        double modifiedJoystickR;
    }

    private final PeriodicIO periodicIO = new PeriodicIO();

    private SystemState currentState = SystemState.IDLE;
    private WantedState wantedState = WantedState.IDLE;

    /*
     * Drivetrain Constants that are in the drivetrain class for ease
     * 
     * Ignore these unless you are cool
     */

    private static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
   .withPigeon2Id(Constants.Drivetrain.pigeonID)
   .withSupportsPro(true)
   .withCANbusName(Constants.Drivetrain.CANBusName);

   private static final SwerveModuleConstantsFactory constantCreator = new SwerveModuleConstantsFactory()
   .withDriveMotorGearRatio(Constants.Drivetrain.driveGearRatio)
   .withSteerMotorGearRatio(Constants.Drivetrain.steerGearRatio)
   .withWheelRadius(Constants.Drivetrain.wheelRadius)
   .withSteerMotorGains(Constants.Drivetrain.steerGains)
   .withDriveMotorGains(Constants.Drivetrain.driveGains)
   .withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
   .withSpeedAt12VoltsMps(Constants.Drivetrain.maxVelocity)
   .withSteerInertia(Constants.Drivetrain.Simulation.angMomentum) // In kg*m^2 SIMULATION
   .withDriveInertia(Constants.Drivetrain.Simulation.driveMometun); // In kg*m^2 SIMULATION

   private static final SwerveModuleConstants frontLeft = constantCreator.createModuleConstants(
        Constants.Drivetrain.Mod0.angleMotorID, Constants.Drivetrain.Mod0.driveMotorID,
        Constants.Drivetrain.Mod0.canCoderID, Constants.Drivetrain.Mod0.dobOffset, Constants.Drivetrain.Mod0.locationX,
        Constants.Drivetrain.Mod0.locationY, Constants.Drivetrain.Mod0.inverted
   );

   private static final SwerveModuleConstants frontRight = constantCreator.createModuleConstants(
        Constants.Drivetrain.Mod1.angleMotorID, Constants.Drivetrain.Mod1.driveMotorID,
        Constants.Drivetrain.Mod1.canCoderID, Constants.Drivetrain.Mod1.dobOffset, Constants.Drivetrain.Mod1.locationX,
        Constants.Drivetrain.Mod1.locationY, Constants.Drivetrain.Mod1.inverted
   );

   private static final SwerveModuleConstants backLeft = constantCreator.createModuleConstants(
        Constants.Drivetrain.Mod2.angleMotorID, Constants.Drivetrain.Mod2.driveMotorID,
        Constants.Drivetrain.Mod2.canCoderID, Constants.Drivetrain.Mod2.dobOffset, Constants.Drivetrain.Mod2.locationX,
        Constants.Drivetrain.Mod2.locationY, Constants.Drivetrain.Mod2.inverted
   );

   private static final SwerveModuleConstants backRight = constantCreator.createModuleConstants(
        Constants.Drivetrain.Mod3.angleMotorID, Constants.Drivetrain.Mod3.driveMotorID,
        Constants.Drivetrain.Mod3.canCoderID, Constants.Drivetrain.Mod3.dobOffset, Constants.Drivetrain.Mod3.locationX,
        Constants.Drivetrain.Mod3.locationY, Constants.Drivetrain.Mod3.inverted
   );
   
    private SwerveDrivetrain ctreframe;

   // END DRIVETRAIN

    private final XboxController controller;
    
    public Drivetrain(XboxController controller) {
        this.controller = controller;
        this.ctreframe = new SwerveDrivetrain(drivetrainConstants,frontLeft,frontRight,backLeft,backRight);
    }

    public void start(){
        wantedState = WantedState.MANUAL;
    }

    @Override
    public void processLoop(double timestamp) {
        SystemState newState;
        switch (currentState) {
            default:
            case IDLE:
                newState = handleStandardState();
                break;
            case MANUAL:
                newState = handleStandardState();
                break;
        }

        if (newState != currentState)
            currentState = newState;
    }

    /*
     * I have an idea for Curve maker thingy for the controller so its not all just linear. If I get around to it, it might come in handy. 
     */
    public void readPeriodicInputs(double timestamp) {
        periodicIO.modifiedJoystickX = controller.getLeftX() * Constants.Drivetrain.maxVelocity;
        periodicIO.modifiedJoystickY = controller.getLeftY() * Constants.Drivetrain.maxVelocity;
        periodicIO.modifiedJoystickR = controller.getRightX() * Constants.Drivetrain.maxRot * 0.75;

    }

    public void writePeriodicOutputs(double timestamp) {
        switch(currentState) {
            case IDLE:
                ctreframe.setControl(Constants.Drivetrain.Requests.idle);
                break;
            case MANUAL:
                ctreframe.setControl(Constants.Drivetrain.Requests.fieldCentric
                .withVelocityX(periodicIO.modifiedJoystickX)
                .withVelocityY(periodicIO.modifiedJoystickY)
                .withRotationalRate(periodicIO.modifiedJoystickR));
                break;
        }
    }

    public SystemState handleStandardState(){
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case MANUAL:
                return SystemState.MANUAL;
        }

        System.err.println("Wanted state of %s is not recognized as a state in drivetrain!".formatted(wantedState.name()));
        
        return null;
    }

    @Override
    public void stop() {
        currentState = SystemState.IDLE;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void zeroSensors() {
        ctreframe.seedFieldRelative(); // Zeros Gyro with +X as forward
    }

    @Override
    public String getId() {
        return "drivetrain";
    }

    public SwerveDrivetrain getRawDrivetrain() {
        return ctreframe;
    }

    
}
