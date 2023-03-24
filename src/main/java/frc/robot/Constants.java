package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.COTSFalconSwerveConstants;
import frc.robot.utils.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final double BEAM_BALANACED_DRIVE_KP = 0.05; // P (Proportional) constant of a PID loop 0.015
    public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 6.8;
    public static final double BALANCEDMAXSPEED = 3;
    public static final double BALANCED_OFFESET = 2.3;

    public static final class Swerve {
       
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK3(6.63);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(17); //TODO: This must be tuned to specific robot 
        public static final double wheelBase = Units.inchesToMeters(22); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.71+180); //293.20);   // 321.4160-180 //141.152
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(326.60-180);  //145.68);  //290.7422-180 //71.455
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 17;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(65.10+180);  //257.50);  //18.3691+180 //295.576
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 18;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(141.60+180);  //320.80);  //53.8767+16.3146 //65.039
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final int NAVX = 19;

    public static final String MANIP_CANBUS = "DADbus";

    public static final double DEADBAND = 0.1;

    
    public final static class DRIVE {
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double TRACKWIDTH_METERS = 20.5; // FIXME Measure and set trackwidth
        public static final double WHEELBASE_METERS = 26.5; // FIXME Measure and set wheelbase


        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(DRIVE.TRACKWIDTH_METERS / 2.0, DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(DRIVE.TRACKWIDTH_METERS / 2.0, -DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(-DRIVE.TRACKWIDTH_METERS / 2.0, DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(-DRIVE.TRACKWIDTH_METERS / 2.0, -DRIVE.WHEELBASE_METERS / 2.0)
        );

        public static final int MAX_FWD_REV_SPEED_MPS_EST = 10;
        public static final int MAX_ROTATE_SPEED_RAD_PER_SEC_EST = 10;
        static public final double MAX_FWD_REV_SPEED_FAST = 0.5; // Percent of output power
        static public final double MAX_STRAFE_SPEED_FAST = 0.5;
        static public final double MAX_ROTATE_SPEED_FAST = 0.35;
        static public final double MAX_FWD_REV_SPEED_SLOW = 0.25; // Percent of output power
        static public final double MAX_STRAFE_SPEED_SLOW = 0.25;
        static public final double MAX_ROTATE_SPEED_SLOW = 0.25;

        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
    }

    public class ARM {
        public static final int EXTENDMOTOR = 13;
		public static final int ROTATEMOTOR = 12;
        public static final int EXTENDENCODER = 20;
        public static final int ROTATEENCODER = 21;

        public static final double MAX_MANUAL_SUPPLY_VOLTAGE = 11.0;
    }
    public final static class INTAKE {
        public static final int INTAKE_MOTOR = 14;

        public static final double MAX_SUPPLY_VOLTAGE = 11.0;
    }
}
