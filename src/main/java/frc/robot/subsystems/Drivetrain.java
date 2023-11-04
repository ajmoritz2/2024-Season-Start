
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenixpro.configs.Slot0Configs;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.CTRSwerve.SwerveDriveConstantsCreator;
import frc.robot.CTRSwerve.SwerveDriveTrainConstants;
import frc.robot.CTRSwerve.SwerveModuleConstants;
import frc.robot.utils.maths.oneDimensionalLookup;

public class Drivetrain implements Subsystem {

    /*
     * DRIVE CONTROLLER SCHEME (PLEASE KEEP UPDATED)
     * 
     * MAPPED BUTTONS:
     * A -> Auto Balance
     * B -> Cruise
     * X -> Limelight Cruise
     * 
     * RBumper -> Lock to 180 degrees
     * LBumper -> Lock to 360 degrees
     * 
     * LAxis -> Drive
     * RAxis -> Rotate
     */

    public static final double MAX_VOLTAGE = 11.0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.36;

    public static double pitchAngle = 0;

    public static double yawToLock = 0;

    private boolean limelightLock = false;
    private boolean limelightDrive = false;

    private double lockDir = 180;
    private boolean lockButton = false; // False is Lbumper True is RBumper

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.DRIVE.MAX_ROTATE_SPEED_RAD_PER_SEC_EST;

    // Actually have no clue what these do. Have too much of a dog brain for this
    public static final double[] XY_Axis_inputBreakpoints = { -1, -0.85, -0.6, -0.12, 0.12, 0.6, 0.85, 1 };
    public static final double[] XY_Axis_outputTable = { -1.0, -0.6, -0.3, 0, 0, 0.3, 0.6, 1.0 };
    public static final double[] RotAxis_inputBreakpoints = { -1, -0.9, -0.6, -0.12, 0.12, 0.6, 0.9, 1 };
    public static final double[] RotAxis_outputTable = { -1.0, -0.5, -0.2, 0, 0, 0.2, 0.5, 1.0 };

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.Swerve.trackWidth / 2.0, Constants.Swerve.wheelBase / 2.0),
            // Front right
            new Translation2d(Constants.Swerve.trackWidth / 2.0, -Constants.Swerve.wheelBase / 2.0),
            // Back left
            new Translation2d(-Constants.Swerve.trackWidth / 2.0, Constants.Swerve.wheelBase / 2.0),
            // Back right
            new Translation2d(-Constants.Swerve.trackWidth / 2.0, -Constants.Swerve.wheelBase / 2.0));
        
    private SwerveDriveOdometry autoOdom;

    private final SlewRateLimiter slewX = new SlewRateLimiter(16);
    private final SlewRateLimiter slewY = new SlewRateLimiter(16);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(1880);

    private double gyroOffset;

    private enum SystemState {
        IDLE,
        MANUAL_CONTROL,
        TRAJECTORY_FOLLOWING, // X,Y axis speeds relative to field
        AUTO_BALANCE,
        LOCK_ROTATION,
        CRUISE,
        LIMELIGHT_CRUISE
    }

    public enum WantedState {
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

        double VxCmd; // longitudinal speed
        double VyCmd; // lateral speed
        double WzCmd; // rotational rate in radians/sec
        boolean robotOrientedModifier; // drive command modifier to set robot oriented translation control

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

    private final PeriodicIO periodicIO = new PeriodicIO();

    private SystemState currentState = SystemState.MANUAL_CONTROL;
    private WantedState wantedState = WantedState.MANUAL_CONTROL;

    private final XboxController controller;

    private double currentStateStartTime;

    private double[] autoDriveSpeeds = new double[2];
    public final static int Num_Modules = 4;
    // public SwerveModule[] mSwerveMods = new SwerveModule[Num_Modules];

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final CTRSwerveDrivetrain drivetrain;

    public SwerveModuleState[] trajectoryStates = new SwerveModuleState[4];

    private boolean balancedX = false, balancedY = false;
    private boolean crawling = false;

    public Drivetrain(XboxController controller) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        // yawCtrl.enableContinuousInput(-Math.PI, Math.PI); //TODO check if Pigeon
        // output rolls over
        
        Slot0Configs steerGains = new Slot0Configs();
        Slot0Configs driveGains = new Slot0Configs();
    
        {
            steerGains.kP = 30;
            steerGains.kD = 0.2;
            driveGains.kP = 2;
        }

        SwerveDriveConstantsCreator constants =  new SwerveDriveConstantsCreator(Constants.Swerve.gearRatio, Constants.Swerve.angleGearRatio, 
        2, 400, steerGains, driveGains, false);
    

        drivetrain = new CTRSwerveDrivetrain(new SwerveDriveTrainConstants().withTurnKp(Constants.Swerve.angleKP).withPigeon2Id(50).withCANbusName("MANIPbus"),

        constants.createModuleConstants(Constants.Swerve.Mod0.angleMotorID, Constants.Swerve.Mod0.driveMotorID, 
                Constants.Swerve.Mod0.canCoderID, Constants.Swerve.Mod0.dobOffset, 0.260, 0.222)
                        ,

                new SwerveModuleConstants().withCANcoderId(Constants.Swerve.Mod1.canCoderID)
                        .withDriveMotorId(Constants.Swerve.Mod1.driveMotorID)
                        .withSteerMotorId(Constants.Swerve.Mod1.angleMotorID)
                        .withCANcoderOffset(Constants.Swerve.Mod1.dobOffset)
                        .withDriveMotorGearRatio(Constants.Swerve.gearRatio)
                        .withSteerMotorGearRatio(Constants.Swerve.angleGearRatio)
                        .withWheelRadius(2)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSlipCurrent(400)
                        .withLocationX(0.260)
                        .withLocationY(-0.222),

                new SwerveModuleConstants().withCANcoderId(Constants.Swerve.Mod2.canCoderID)
                        .withDriveMotorId(Constants.Swerve.Mod2.driveMotorID)
                        .withSteerMotorId(Constants.Swerve.Mod2.angleMotorID)
                        .withCANcoderOffset(Constants.Swerve.Mod2.dobOffset)
                        .withDriveMotorGearRatio(Constants.Swerve.gearRatio)
                        .withSteerMotorGearRatio(Constants.Swerve.angleGearRatio)
                        .withWheelRadius(2)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSlipCurrent(400)
                        .withLocationX(-0.260)
                        .withLocationY(0.222),

                new SwerveModuleConstants().withCANcoderId(Constants.Swerve.Mod3.canCoderID)
                        .withDriveMotorId(Constants.Swerve.Mod3.driveMotorID)
                        .withSteerMotorId(Constants.Swerve.Mod3.angleMotorID)
                        .withCANcoderOffset(Constants.Swerve.Mod3.dobOffset)
                        .withDriveMotorGearRatio(Constants.Swerve.gearRatio)
                        .withSteerMotorGearRatio(Constants.Swerve.angleGearRatio)
                        .withWheelRadius(2)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSlipCurrent(400)
                        .withLocationX(-0.260)
                        .withLocationY(-0.222) );

        // mSwerveMods = new SwerveModule[] {
        // new SwerveModule(0, Constants.Swerve.Mod0.constants),
        // new SwerveModule(1, Constants.Swerve.Mod1.constants),
        // new SwerveModule(2, Constants.Swerve.Mod2.constants),
        // new SwerveModule(3, Constants.Swerve.Mod3.constants)
        // };

        drivetrain.seedFieldRelative();
        autoOdom = drivetrain.getOdometry();

        this.controller = controller;

    }

    @Override
    public void processLoop(double timestamp) {
        SystemState newState;
        switch (currentState) {
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

    }

    @Override
    public void readPeriodicInputs(double timestamp) {

        periodicIO.VxCmd = -oneDimensionalLookup.interpLinear(XY_Axis_inputBreakpoints, XY_Axis_outputTable,
                controller.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND;
        periodicIO.VyCmd = -oneDimensionalLookup.interpLinear(XY_Axis_inputBreakpoints, XY_Axis_outputTable,
                controller.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND;
        periodicIO.WzCmd = -oneDimensionalLookup.interpLinear(RotAxis_inputBreakpoints, RotAxis_outputTable,
                controller.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        periodicIO.robotOrientedModifier = controller.getLeftTriggerAxis() > 0.25;

        // periodicIO.modifiedJoystickX = -slewX
        //         .calculate(-controller.getLeftX() * halfWhenCrawl(MAX_VELOCITY_METERS_PER_SECOND));
        // periodicIO.modifiedJoystickY = -slewY
        //         .calculate(-controller.getLeftY() * halfWhenCrawl(MAX_VELOCITY_METERS_PER_SECOND));

        periodicIO.modifiedJoystickX = controller.getLeftX() * halfWhenCrawl(MAX_VELOCITY_METERS_PER_SECOND);
        periodicIO.modifiedJoystickY = controller.getLeftY() * halfWhenCrawl(MAX_VELOCITY_METERS_PER_SECOND);

        if (limelightLock) {
            periodicIO.modifiedJoystickX = slewX
                    .calculate(MathUtil.clamp(-controller.getLeftX() * halfWhenCrawl(MAX_VELOCITY_METERS_PER_SECOND),
                            -Constants.DRIVE.CRUISING_SPEED, Constants.DRIVE.CRUISING_SPEED));
            periodicIO.modifiedJoystickY = -slewY
                    .calculate(MathUtil.clamp(-controller.getLeftY() * halfWhenCrawl(MAX_VELOCITY_METERS_PER_SECOND),
                            -Constants.DRIVE.CRUISING_SPEED, Constants.DRIVE.CRUISING_SPEED));
        }
        periodicIO.modifiedJoystickR = -slewRot
                .calculate(-controller.getRightX() * halfWhenCrawl(MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)) * 0.75;

        checkButtons();

    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        ChassisSpeeds chassis = new ChassisSpeeds(0,0,0);
        //Logger.getInstance().recordOutput("ACtual Pose", drivetrain.getOdometry().getPoseMeters());

        switch (currentState) {
            case TRAJECTORY_FOLLOWING:
                SwerveDriveKinematics.desaturateWheelSpeeds(trajectoryStates, Constants.Swerve.maxSpeed);
                //Logger.getInstance().recordOutput("Swerve Module States", trajectoryStates);
                chassis = drivetrain.getKinematics().toChassisSpeeds(trajectoryStates[0], trajectoryStates[1], trajectoryStates[2], trajectoryStates[3]
                );
                break;
            case AUTO_BALANCE:
                autoBalance();
                break;


            case LIMELIGHT_CRUISE:
                if (limelightLock)
                    chassis = new ChassisSpeeds(
                            MathUtil.clamp(periodicIO.
                            modifiedJoystickY, -Constants.DRIVE.CRUISING_SPEED,
                                    Constants.DRIVE.CRUISING_SPEED),
                            MathUtil.clamp(periodicIO.modifiedJoystickX, -Constants.DRIVE.CRUISING_SPEED,
                                    Constants.DRIVE.CRUISING_SPEED),
                            periodicIO.modifiedJoystickR);

                break;
            case LOCK_ROTATION:
                if (lockButton)
                    chassis = new ChassisSpeeds(periodicIO.modifiedJoystickY, periodicIO.modifiedJoystickX,
                            correctRightRotation(lockDir));
                else
                    chassis = new ChassisSpeeds(periodicIO.modifiedJoystickY, periodicIO.modifiedJoystickX,
                            correctLeftRotation(lockDir));

                break;
            case CRUISE:
                chassis = new ChassisSpeeds(Constants.DRIVE.CRUISING_SPEED, 0, periodicIO.modifiedJoystickR);
                break;
            case MANUAL_CONTROL:
                chassis = new ChassisSpeeds(periodicIO.modifiedJoystickY, periodicIO.modifiedJoystickX,
                        periodicIO.modifiedJoystickR);
                break;
            default:
            case IDLE:
                break;

        }

        //SmartDashboard.putNumber("Drivetrain/Chassis X", chassis.vxMetersPerSecond);
        //SmartDashboard.putNumber("Drivetrain/Chassis Y", chassis.vyMetersPerSecond);
        //SmartDashboard.putNumber("Drivetrain/Chassis Angle", chassis.omegaRadiansPerSecond);

        drivetrain.driveFieldCentric(chassis);
        // updateStateVariables(moduleStates);
    }

    private SystemState defaultStateChange() {
        switch (wantedState) {
            /*
             * case IDLE:
             * return SystemState.IDLE;
             */
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

    private void checkButtons() {
        if (controller.getAButtonPressed())
            setWantedState(WantedState.AUTO_BALANCE);
        if (controller.getAButtonReleased())
            setWantedState(WantedState.MANUAL_CONTROL);
            
        if (controller.getBButtonPressed()){
            //zeroSensors();
            zeroGyroscope();
        }

        pitchAngle = drivetrain.getPitch() - Constants.BALANCED_OFFESET;

        if (Math.abs(pitchAngle) >= Math.abs(Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES) && balancedX) {
            balancedX = false;
        } else if (!balancedX && Math.abs(pitchAngle) <= Math.abs(Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES)) {
            balancedX = true;
        }

        if (controller.getRightBumper())
            limelightLock = true;
        else
            limelightLock = false;

        if (getLeftTrigger()) {
            setWantedState(WantedState.LOCK_ROTATION);
            lockDir = 180;
            lockButton = true;
        } else if (controller.getLeftBumper()) {
            setWantedState(WantedState.LOCK_ROTATION);
            lockDir = 360;
            lockButton = false;
        } else if (currentState == SystemState.LOCK_ROTATION) {
            setWantedState(WantedState.MANUAL_CONTROL);
        }
    }

    private double lockToPi(double value) {
        return ((value >= Math.PI * 2)) ? value - (Math.PI * 2) : value;
    }

    private double correctRightRotation(double goal) {
        double steeringAdjust = 0;
        final double heading_error = (getYaw().getRadians() - Math.toRadians(goal));
        final double Kp = 0.000001;
        final double min_command = 1;

        double correctedError = heading_error - Math.PI;

        if (Math.abs(heading_error) > 0.1) {
            if (heading_error > Math.PI)
                steeringAdjust = Kp * correctedError + min_command; // positive
            else
                steeringAdjust = Kp * correctedError - min_command; // negative TEST THIS ONE FIRST!!!
        } else {
            steeringAdjust = 0;
        }
        return steeringAdjust * 2;
    }

    private double correctLeftRotation(double goal) {
        double steeringAdjust = 0;
        final double heading_error = (getYaw().getRadians() - Math.toRadians(goal));
        final double Kp = 0.00001;
        final double min_command = 1;

        if (Math.abs(heading_error) > 0.1) {
            if (heading_error < 0)
                steeringAdjust = heading_error * Kp + min_command; // positive
            else
                steeringAdjust = heading_error * Kp - min_command; // negative
        } else {
            steeringAdjust = 0;
        }
        return steeringAdjust * 2;
    }

    private double halfWhenCrawl(double val) {
        return (crawling) ? val / 2 : val;
    }

    private void autoBalance() {
        // TODO: Cap the angles given so we never calculate above a certain value.
        double xAxisRate = 0;

        if (!balancedX && pitchAngle > 0) {

            double pitchAngleRadians = pitchAngle * (Math.PI / 180.0);
            xAxisRate = Math.min(4, Math.abs(Math.sin(pitchAngleRadians)));
        }

        if (!balancedX && pitchAngle < 0) {

            double pitchAngleRadians = pitchAngle * (Math.PI / 180.0);
            xAxisRate = Math.min(4, Math.abs(Math.sin(pitchAngleRadians)) * -0.3);
        }

        drive(xAxisRate * Constants.BALANCEDMAXSPEED, 0, 0.0, true);

    }

    @Override
    public void zeroSensors() {
        drivetrain.resetOdometry();
        drivetrain.driveFullyFieldCentric(0, 0, getYaw());
    }

    @Override
    public void stop() {
        drivetrain.driveFullyFieldCentric(0, 0, getYaw());
    }

    @Override
    public String getId() {
        return "Drivetrain";
    }

    public boolean getLeftTrigger() {
        return (controller.getRawAxis(2) == 1) ? true : false;
    }

    public boolean getRightTrigger() {
        return (controller.getRawAxis(3) == 1) ? true : false;
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
        // SmartDashboard.putString("drivetrain/currentStateOutputs",
        // currentState.toString());
        // SmartDashboard.putString("drivetrain/wantedStateOutputs",
        // wantedState.toString());
        // SmartDashboard.putNumber("drivetrain/goalLimelightAngleError",
        // periodicIO.limelightAngleError);
        // SmartDashboard.putNumber("drivetrain/yawAngle", periodicIO.adjustedYaw);
        // SmartDashboard.putString("drivetrain/pose", getPose().toString());
        // SmartDashboard.putNumber("drivetrain/chassisVx", periodicIO.chassisVx);
        // SmartDashboard.putNumber("drivetrain/chassisVy", periodicIO.chassisVy);
        // SmartDashboard.putNumber("drivetrain/goalVx", periodicIO.goalVx);
        // SmartDashboard.putNumber("drivetrain/goalVy", periodicIO.goalVy);

        // SmartDashboard.putNumber("PosX",
        // odometry.getPoseMeters().getTranslation().getX());
        // SmartDashboard.putNumber("PosY",
        // odometry.getPoseMeters().getTranslation().getY());
        //SmartDashboard.putNumber("Yaw", getYaw().getRadians());
        //SmartDashboard.putNumber("Goal Angle", (getYaw().getRadians() - Math.toRadians(yawToLock)));
        //SmartDashboard.putNumber("Pitch", pitchAngle);
        //SmartDashboard.putNumber("Goal switch", lockToPi(Math.toRadians(yawToLock) + Math.PI * 2));

    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void initAutonPosition(PathPlannerTrajectory.PathPlannerState state) {
        // ErrorCode errorCode = pigeon.setYaw(state.holonomicRotation.getDegrees(),
        // 100);
        drivetrain.getOdometry().resetPosition(getYaw(), getModulePositions(),
                new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
    }

    public void zeroGyroscope() {
        drivetrain.seedFieldRelative();
        gyroOffset = 0;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        ChassisSpeeds chassis = drivetrain.getKinematics().toChassisSpeeds(desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]
        );

        drivetrain.driveFieldCentric(chassis);
    }

    public void setModuleStatesFromTrajectory(SwerveModuleState[] states) {
        
        trajectoryStates = states;
    }

    private SystemState handleManualControl() {

        return defaultStateChange();
    }

    private SystemState handleTrajectoryFollowing() {
        return defaultStateChange();
    }

    public Pose2d getPose() {
        return drivetrain.getPoseMeters();
    }



    public void setAutoDriveSpeeds(double xSpeed, double ySpeed) {
        autoDriveSpeeds[0] = xSpeed;
        autoDriveSpeeds[1] = ySpeed;
    }

    public CTRSwerveDrivetrain getBaseDrivetrain(){
        return drivetrain;
    }

    public SwerveModulePosition[] getModulePositions() {

        return drivetrain.getSwervePositions();
    }


    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
        //         : new ChassisSpeeds(xSpeed, ySpeed, rot);
        // SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        // for (SwerveModule mod : mSwerveMods) {
        //     mod.setDesiredState(states[mod.moduleNumber], true);
        // }
        // return states;
        drivetrain.driveFieldCentric(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }

    public Rotation2d getYaw() {

        return Rotation2d.fromDegrees(drivetrain.getYaw());
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }
}