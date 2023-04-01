
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.SwerveModule;
import frc.robot.Constants.*;
import frc.robot.utils.maths.oneDimensionalLookup;

public class Drivetrain implements Subsystem {
    
    /*  DRIVE CONTROLLER SCHEME (PLEASE KEEP UPDATED)
    * 
     * MAPPED BUTTONS:
     *  A -> Auto Balance
     *  B -> Cruise
     *  X -> Limelight Cruise
     * 
     *  RBumper -> Lock to 180 degrees
     *  LBumper -> Lock to 360 degrees
     * 
     *  LAxis -> Drive
     *  RAxis -> Rotate 
     */
  
    public static final double MAX_VOLTAGE = 11.0;
  
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.36;

    public static double pitchAngle = 0;

    private double lockDir = 180;
    private boolean lockButton = false; // False is Lbumper True is RBumper
 
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.DRIVE.MAX_ROTATE_SPEED_RAD_PER_SEC_EST;

    // Actually have no clue what these do. Have too much of a dog brain for this
    public static final double[] XY_Axis_inputBreakpoints = {-1, -0.85, -0.6, -0.12, 0.12, 0.6, 0.85, 1};
    public static final double[] XY_Axis_outputTable = {-1.0, -0.6, -0.3, 0, 0, 0.3, 0.6, 1.0};
    public static final double[] RotAxis_inputBreakpoints = {-1, -0.9, -0.6, -0.12, 0.12, 0.6, 0.9, 1};
    public static final double[] RotAxis_outputTable = {-1.0, -0.5, -0.2, 0, 0, 0.2, 0.5, 1.0};

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(Constants.Swerve.trackWidth / 2.0, Constants.Swerve.wheelBase / 2.0),
        // Front right
        new Translation2d(Constants.Swerve.trackWidth / 2.0, -Constants.Swerve.wheelBase / 2.0),
        // Back left
        new Translation2d(-Constants.Swerve.trackWidth / 2.0, Constants.Swerve.wheelBase / 2.0),
        // Back right
        new Translation2d(-Constants.Swerve.trackWidth / 2.0, -Constants.Swerve.wheelBase / 2.0)
    );

    private final SlewRateLimiter slewX = new SlewRateLimiter(16);
    private final SlewRateLimiter slewY = new SlewRateLimiter(16);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(1880);

 
    private final AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) 200);
    private double gyroOffset;


    private enum SystemState{
        IDLE,   
        MANUAL_CONTROL,
        TRAJECTORY_FOLLOWING,           //X,Y axis speeds relative to field
        AUTO_BALANCE,
        LOCK_ROTATION,
        CRUISE,
        LIMELIGHT_CRUISE
    }

    public enum WantedState{
        IDLE,
        MANUAL_CONTROL,
        TRAJECTORY_FOLLOWING,
        AUTO_BALANCE,
        LOCK_ROTATION,
        CRUISE,
        LIMELIGHT_CRUISE
    }

    private static class PeriodicIO {
        // INPUTS
        double timestamp;

        double VxCmd; //longitudinal speed
        double VyCmd; //lateral speed
        double WzCmd; //rotational rate in radians/sec
        boolean robotOrientedModifier; //drive command modifier to set robot oriented translation control

        double modifiedJoystickX;
        double modifiedJoystickY;
        double modifiedJoystickR;

        double limelightAngleError;
        double limelightDistance;

        double yawRateMeasured;
        double adjustedYaw;

        double chassisVx;
        double chassisVy;
        double goalVx;
        double goalVy;

        // OUTPUTS
        SwerveModuleState[] swerveStatesOut;
    }

    private static class StateVariables {
        double frontLeftDriveLast;
        double frontLeftAngleLast;
        double frontRightDriveLast;
        double frontRightAngleLast;
        double rearLeftDriveLast;
        double rearLeftAngleLast;
        double rearRightDriveLast;
        double rearRightAngleLast;
    }

    private final PeriodicIO periodicIO = new PeriodicIO();
    private final StateVariables stateVariables = new StateVariables();

    private SystemState currentState = SystemState.MANUAL_CONTROL;
    private WantedState wantedState = WantedState.MANUAL_CONTROL;

    private final XboxController controller;

    private double currentStateStartTime;

    private double[] autoDriveSpeeds = new double[2];
    public final static int Num_Modules = 4;
    public SwerveModule[] mSwerveMods = new SwerveModule[Num_Modules];   

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final SwerveDriveOdometry odometry; 
    
    public SwerveModuleState[] trajectoryStates = new SwerveModuleState[4];

    private boolean balancedX = false, balancedY = false;
    private boolean crawling = false;

    public Drivetrain(XboxController controller) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        //yawCtrl.enableContinuousInput(-Math.PI, Math.PI);  //TODO check if Pigeon output rolls over 

        
        
        mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        
        odometry = new SwerveDriveOdometry(m_kinematics, getYaw(), getModulePositions());

        this.controller = controller;

        resetModulesToAbsolute();
        

    }

    @Override
    public void processLoop(double timestamp) {
        SystemState newState;
        switch (currentState){
            default:
            case MANUAL_CONTROL:
                newState = handleManualControl();
                break;
            case AUTO_BALANCE:
                newState = handleManualControl();
                break;
            case TRAJECTORY_FOLLOWING:
                newState = handleTrajectoryFollowing();
                break;
            case LOCK_ROTATION:
                newState = handleManualControl();
                break;
            case CRUISE:
                newState = handleManualControl();
                break;
            case LIMELIGHT_CRUISE:
                newState = handleManualControl();
                break;
            case IDLE:
                newState = handleManualControl();
                break;
      
        }
        if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
		}
        
        updateOdometry();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {

        periodicIO.VxCmd = -oneDimensionalLookup.interpLinear(XY_Axis_inputBreakpoints, XY_Axis_outputTable, controller.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND;
        periodicIO.VyCmd = -oneDimensionalLookup.interpLinear(XY_Axis_inputBreakpoints, XY_Axis_outputTable, controller.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND;
        periodicIO.WzCmd = -oneDimensionalLookup.interpLinear(RotAxis_inputBreakpoints, RotAxis_outputTable, controller.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        periodicIO.robotOrientedModifier = controller.getLeftTriggerAxis() > 0.25;

        periodicIO.modifiedJoystickX = slewX.calculate(-controller.getLeftX() * halfWhenCrawl(MAX_VELOCITY_METERS_PER_SECOND));
        periodicIO.modifiedJoystickY = slewY.calculate(-controller.getLeftY() * halfWhenCrawl(MAX_VELOCITY_METERS_PER_SECOND));
        periodicIO.modifiedJoystickR = slewRot.calculate(-controller.getRightX() * halfWhenCrawl(MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND))*0.75;
        

        double[] chassisVelocity = chassisSpeedsGetter();
        periodicIO.chassisVx = chassisVelocity[0];
        periodicIO.chassisVy = chassisVelocity[1];
        periodicIO.goalVx = chassisVelocity[2];
        periodicIO.goalVy = chassisVelocity[3];
        
        checkButtons();

    }

    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        switch(currentState){
            case TRAJECTORY_FOLLOWING:
                moduleStates = trajectoryStates;
                break;
            case AUTO_BALANCE:
                moduleStates = autoBalance();
                //System.out.println("IN balance");
                break;
            case LIMELIGHT_CRUISE:
                moduleStates = drive(0, Constants.DRIVE.CRUISING_SPEED, Robot.m_robotContainer.limelight.steeringAdjust(), false);
                break;
            case LOCK_ROTATION:
                if(lockButton)
                    moduleStates = drive(periodicIO.modifiedJoystickY,periodicIO.modifiedJoystickX, correctRightRotation(lockDir), !periodicIO.robotOrientedModifier);
                else
                    moduleStates = drive(periodicIO.modifiedJoystickY,periodicIO.modifiedJoystickX, correctLeftRotation(lockDir), !periodicIO.robotOrientedModifier);

                break;
            case CRUISE:
                moduleStates = drive(Constants.DRIVE.CRUISING_SPEED, 0, periodicIO.modifiedJoystickR, false);
                break;
            case MANUAL_CONTROL:
                moduleStates = drive(periodicIO.modifiedJoystickY, periodicIO.modifiedJoystickX, periodicIO.modifiedJoystickR, !periodicIO.robotOrientedModifier);
                break;
            default:
            case IDLE:
                moduleStates = drive(0.0, 0.0, 0.0, true);
                break;

        }


        setModuleStates(moduleStates);
        updateStateVariables(moduleStates);
    }

    
    private SystemState defaultStateChange() {
		switch (wantedState){
            /*case IDLE:
                return SystemState.IDLE;*/
            case AUTO_BALANCE:
                return SystemState.AUTO_BALANCE;
            case TRAJECTORY_FOLLOWING:
				return SystemState.TRAJECTORY_FOLLOWING;
            default:
            case MANUAL_CONTROL:
                return SystemState.MANUAL_CONTROL;
            case LOCK_ROTATION:
                return SystemState.LOCK_ROTATION;
            case CRUISE:
                return SystemState.CRUISE;
            case LIMELIGHT_CRUISE:
                return SystemState.LIMELIGHT_CRUISE;
		}
	}

    @Override
    public void periodic() {
        
    }

    private void checkButtons(){
        if(controller.getAButtonPressed())
            setWantedState(WantedState.AUTO_BALANCE);
        if(controller.getAButtonReleased())
            setWantedState(WantedState.MANUAL_CONTROL);

        if(controller.getBButtonPressed())
            setWantedState(WantedState.CRUISE);
        if(controller.getBButtonReleased())
            setWantedState(WantedState.MANUAL_CONTROL);

        if(controller.getXButtonPressed())
            setWantedState(WantedState.LIMELIGHT_CRUISE);
        if(controller.getXButtonReleased())
            setWantedState(WantedState.MANUAL_CONTROL);
        
        pitchAngle = ahrs.getPitch() - Constants.BALANCED_OFFESET;

        
        if (Math.abs(pitchAngle) >= Math.abs(Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES) && balancedX){
            balancedX = false;
        } else if (!balancedX && Math.abs(pitchAngle) <= Math.abs(Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES)){
            balancedX = true;
        }

        if (controller.getRightBumper()){
            setWantedState(WantedState.LOCK_ROTATION);
            lockDir = 180;
            lockButton = true;
        } else if (controller.getLeftBumper()){
            setWantedState(WantedState.LOCK_ROTATION);
            lockDir = 360;
            lockButton = false;
        }else if (currentState == SystemState.LOCK_ROTATION){
            setWantedState(WantedState.MANUAL_CONTROL);
        }
    }

    /** 
     * Need to test this function
     * 
     * @param goal the goal angle
     *  @return whether the stuff works
     */
    private double correctAllRotation(double goal) {
        double steeringAdjust = 0;
        double heading_error = (getYaw().getRadians() - Math.toRadians(goal));
        final double Kp = 0.0001;
        final double min_command = 1;
    
        if(Math.abs(heading_error)>0.1){
            if(heading_error<lockToPi(goal + Math.PI))
              steeringAdjust = heading_error*Kp + min_command; // positive
            else
              steeringAdjust = heading_error*Kp - min_command; // negative
        } else {
            steeringAdjust = 0;
        }
        return steeringAdjust*2;
    }

    private double lockToPi(double value){
        return ((value >= Math.PI*2)) ? value-(Math.PI*2) : value;
    }

    private double correctRightRotation(double goal){
        double steeringAdjust = 0;
        final double heading_error = (getYaw().getRadians()- Math.toRadians(goal));
        final double Kp = 0.0001;
        final double min_command = 1;

        double correctedError = heading_error - Math.PI;
    
        if(Math.abs(heading_error)>0.1){
            if(heading_error>Math.PI)
              steeringAdjust = Kp*correctedError + min_command; //positive
            else
              steeringAdjust = Kp*correctedError - min_command; //negative TEST THIS ONE FIRST!!!
        } else {
            steeringAdjust = 0;
        }
        return steeringAdjust*2;
    }

    private double correctLeftRotation(double goal){
        double steeringAdjust = 0;
        final double heading_error = (getYaw().getRadians()- Math.toRadians(goal));
        final double Kp = 0.0001;
        final double min_command = 1;
    
        if(Math.abs(heading_error)>0.1){
            if(heading_error<0)
              steeringAdjust = heading_error*Kp + min_command; // positive
            else
              steeringAdjust = heading_error*Kp - min_command; // negative
        } else {
            steeringAdjust = 0;
        }
        return steeringAdjust*2;
    }

    private double halfWhenCrawl(double val){
        return (crawling) ? val/2 : val;
    }

    private SwerveModuleState[] autoBalance(){
        // TODO: Cap the angles given so we never calculate above a certain value.
        double xAxisRate = 0;         
        


        if (!balancedX && pitchAngle > 0) {

            double pitchAngleRadians = pitchAngle * (Math.PI / 180.0);
            xAxisRate = Math.min(4, Math.abs(Math.sin(pitchAngleRadians)));
        }

        if (!balancedX && pitchAngle < 0) {

            double pitchAngleRadians = pitchAngle * (Math.PI / 180.0);
            xAxisRate =  Math.min(4, Math.abs(Math.sin(pitchAngleRadians))*-0.3);
        }

        return drive(xAxisRate*Constants.BALANCEDMAXSPEED, 0, 0.0, true);
 
        
    }

    private double[] chassisSpeedsGetter(){
        double[] speeds = new double[4];
        int i = 0;
        for (SwerveModule m : mSwerveMods){
            speeds[i] = m.getState().speedMetersPerSecond;
            i++;
        }
        return speeds;
    }

    private void updateStateVariables(SwerveModuleState[] states)
    {
        stateVariables.frontLeftDriveLast = states[0].speedMetersPerSecond;
        stateVariables.frontRightDriveLast = states[1].speedMetersPerSecond;
        stateVariables.rearLeftDriveLast = states[2].speedMetersPerSecond;
        stateVariables.rearRightDriveLast = states[3].speedMetersPerSecond;

        stateVariables.frontLeftAngleLast = states[0].angle.getRadians();
        stateVariables.frontRightAngleLast = states[1].angle.getRadians();
        stateVariables.rearLeftAngleLast = states[2].angle.getRadians();
        stateVariables.rearRightAngleLast = states[3].angle.getRadians();
    }

    @Override
    public void zeroSensors(){
        resetOdometry();
        setModuleStates(drive(0.0, 0.0, 0.0, true));
    }

    @Override
    public void stop(){
        setModuleStates(drive(0.0, 0.0, 0.0, true));
    }

    @Override
    public String getId() {
        return "Drivetrain";
    }

    @Override
    public void outputTelemetry(double timestamp) {
        
        // SmartDashboard.putString("drivetrain/currentState", currentState.toString());
        // SmartDashboard.putString("drivetrain/wantedState", wantedState.toString());
        // SmartDashboard.putBoolean("drivetrain/balancedX", balancedX);
        // SmartDashboard.putNumber("ModifiedX", periodicIO.modifiedJoystickX);
        // SmartDashboard.putNumber("ModifiedY", periodicIO.modifiedJoystickY);
        // SmartDashboard.putNumber("ModifiedR", periodicIO.modifiedJoystickR);
        // SmartDashboard.putString("drivetrain/currentStates", currentState.name());
        // SmartDashboard.putNumber("drivetrain/heading",periodicIO.adjustedYaw);
        // SmartDashboard.putString("drivetrain/pose",odometry.getPoseMeters().toString());
        // SmartDashboard.putString("drivetrain/currentState", currentState.toString());
        // SmartDashboard.putString("drivetrain/wantedState", wantedState.toString());
        // SmartDashboard.putNumber("drivetrain/VxCmd", periodicIO.VxCmd);
        // SmartDashboard.putNumber("drivetrain/VyCmd", periodicIO.VyCmd);
        // SmartDashboard.putNumber("drivetrain/WzCmd", periodicIO.WzCmd);
        // SmartDashboard.putNumber("driverController/LeftY", controller.getLeftY());
        // SmartDashboard.putNumber("driverController/LeftX", controller.getLeftX());
        // SmartDashboard.putNumber("driverController/RightX", controller.getRightX());
        // SmartDashboard.putString("drivetrain/currentStateOutputs", currentState.toString());
        // SmartDashboard.putString("drivetrain/wantedStateOutputs", wantedState.toString());
        // SmartDashboard.putNumber("drivetrain/goalLimelightAngleError", periodicIO.limelightAngleError);
        // SmartDashboard.putNumber("drivetrain/yawAngle", periodicIO.adjustedYaw);
        // SmartDashboard.putString("drivetrain/pose", getPose().toString());
        // SmartDashboard.putNumber("drivetrain/chassisVx", periodicIO.chassisVx);
        // SmartDashboard.putNumber("drivetrain/chassisVy", periodicIO.chassisVy);
        // SmartDashboard.putNumber("drivetrain/goalVx", periodicIO.goalVx);
        // SmartDashboard.putNumber("drivetrain/goalVy", periodicIO.goalVy);
        
        // SmartDashboard.putNumber("PosX", odometry.getPoseMeters().getTranslation().getX());
        // SmartDashboard.putNumber("PosY", odometry.getPoseMeters().getTranslation().getY());
        
        SmartDashboard.putNumber("Yaw", getYaw().getRadians());
        SmartDashboard.putNumber("Goal Angle", lockDir);
        SmartDashboard.putNumber("Pitch", pitchAngle);


        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        
    }

    public void setWantedState(WantedState wantedState)
    {
        this.wantedState = wantedState;
    }

    public void initAutonPosition(PathPlannerTrajectory.PathPlannerState state){
        zeroGyroscope();
        //ErrorCode errorCode = pigeon.setYaw(state.holonomicRotation.getDegrees(), 100);
        odometry.resetPosition(getYaw(), getModulePositions(), new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
    }

    public void zeroGyroscope() {
        ahrs.zeroYaw();
    
        gyroOffset = 0;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    } 

    public void setModuleStatesFromTrajectory(SwerveModuleState[] states){
        trajectoryStates = states;
    }
    
    private SystemState handleManualControl(){

        return defaultStateChange();
    }

    private SystemState handleTrajectoryFollowing(){
        return defaultStateChange();
    }

    private boolean checkNoDriveInput(){
        return(Math.abs(periodicIO.VxCmd) < Constants.DEADBAND
                && Math.abs(periodicIO.VyCmd) < Constants.DEADBAND
                && Math.abs(periodicIO.WzCmd) < Constants.DEADBAND);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    public void resetPose(Pose2d odo){
        odometry.resetPosition(getYaw(), getModulePositions(), odo);;
    }


    private void updateOdometry(){
        odometry.update(getYaw(), getModulePositions());  
    }

    public void resetOdometry(){
        zeroGyroscope();
        odometry.resetPosition(getYaw(), getModulePositions(), new Pose2d(0,0, new Rotation2d()));;
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public void setAutoDriveSpeeds(double xSpeed, double ySpeed){
        autoDriveSpeeds[0] = xSpeed;
        autoDriveSpeeds[1] = ySpeed;
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i<4; i++){
            positions[i] = mSwerveMods[i].getPosition();
        }
        return positions;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(states[mod.moduleNumber], true);
        }
        return states;
    }
 

    public Rotation2d getYaw() {
        if (ahrs.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(ahrs.getFusedHeading());
          }
       //
       //    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
       
          return Rotation2d.fromDegrees(360.0 - ahrs.getYaw());
    }

    public double getPitch() {
        //
        //    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
           return ahrs.getPitch();
     }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }
}