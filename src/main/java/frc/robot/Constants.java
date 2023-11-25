package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

public final class Constants {

    public static final double loopLengthSeconds = 0.02;

    public final static class Controllers {
        public static final int driveID = 0;
    }

    public final static class Drivetrain {

        public static final int pigeonID = 50;

        public static final String CANBusName = "MANIPbus";

        public static final double maxVelocity = 5; // m/s
        public static final double maxRot = Math.PI; // r/s

        public static final double driveGearRatio = 6.63;
        public static final double steerGearRatio = 1; // Check

        public static final double wheelRadius = 2; // in Inches

        public static final Slot0Configs steerGains = new Slot0Configs();
        public static final Slot0Configs driveGains = new Slot0Configs();
    
        {
            steerGains.kP = 30;
            steerGains.kD = 0.2;
            driveGains.kP = 2;
        }

        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final double locationX = 0.266; // In m
            public static final double locationY = 0.222; // In m
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 15;
            public static final double dobOffset = 0.015+0.994141-1.320312; // In rad

            public static final boolean inverted = false; // For drive
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final double locationX = 0.266;
            public static final double locationY = -0.222;
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 16;
            public static final double dobOffset = -0.692871;

            public static final boolean inverted = false;
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final double locationX = -0.266;
            public static final double locationY = 0.222;
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 17;
            public static final double dobOffset = -0.682129;

            public static final boolean inverted = false;
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final double locationX = -0.266;
            public static final double locationY = -0.222;
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 18;
            public static final double dobOffset = -0.38916; 

            public static final boolean inverted = false;
        }

        public static final class Simulation {
            public static final double angMomentum = 4.3; // kg*m^2
            public static final double driveMometun = 5.4; // kg*m^2
        }

        public static final class Requests { // Can create own using the SwerveRequest Interface. Not nessecary unless ultra advanced!!!
            public static final FieldCentric fieldCentric = new FieldCentric().withIsOpenLoop(true);
            public static final SwerveRequest.Idle idle = new SwerveRequest.Idle().withIsOpenLoop(true);
        }
    }
    
}
