package frc.robot.auton;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.SystemState;
import frc.robot.subsystems.Drivetrain.WantedState;
import frc.robot.auton.commands.*;

public class Autons {
    private static List<PathPlannerTrajectory> center = PathPlanner.loadPathGroup("center", new PathConstraints(5, 4));
    private static List<PathPlannerTrajectory> clear = PathPlanner.loadPathGroup("Clear", new PathConstraints(2.5, 3));
    private static List<PathPlannerTrajectory> wireCover = PathPlanner.loadPathGroup("WireCover", new PathConstraints(2.5, 3));
    
    public static Command center(Drivetrain driveTrain, Arm arm, Intake intake){
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        // // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //     driveTrain::getPose, // Pose2d supplier// Pose2d consumer, used to reset odometry at the beginning of auto
        //     driveTrain::resetPose,
        //     driveTrain.getKinematics(), // SwerveDriveKinematics
        //     new PIDConstants(.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        //     new PIDConstants(0.6, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        //     driveTrain::setModuleStatesFromTrajectory, // Module states consumer used to output to the drive subsystem
        //     eventMap,
        //     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        //     driveTrain // The drive subsystem. Used to properly set the requirements of path following commands
        // );
        // driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING);

        Command fullAuto = generateFullAuto(center, driveTrain);

    
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(1.2),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto.alongWith(new WaitCommand(2)),
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new InstantCommand(() -> driveTrain.setWantedState(Drivetrain.WantedState.AUTO_BALANCE))
            );
    }

    public static Command clear(Drivetrain driveTrain, Arm arm, Intake intake){
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        
        // // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //     driveTrain::getPose, // Pose2d supplier// Pose2d consumer, used to reset odometry at the beginning of auto
        //     driveTrain::resetPose,
        //     driveTrain.getKinematics(), // SwerveDriveKinematics
        //     new PIDConstants(.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        //     new PIDConstants(0.6, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        //     driveTrain::setModuleStatesFromTrajectory, // Module states consumer used to output to the drive subsystem
        //     eventMap,
        //     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        //     driveTrain // The drive subsystem. Used to properly set the requirements of path following commands
        // );
        // driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING);
        
        Command fullAuto = generateFullAuto(clear, driveTrain);
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(1.2),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto.alongWith(new WaitCommand(5)),
                new InstantCommand(() -> driveTrain.setWantedState(Drivetrain.WantedState.AUTO_BALANCE))
            );
    }

    public static Command wireCover(Drivetrain driveTrain, Arm arm, Intake intake){
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        
        // // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //     driveTrain::getPose, // Pose2d supplier// Pose2d consumer, used to reset odometry at the beginning of auto
        //     driveTrain::resetPose,
        //     driveTrain.getKinematics(), // SwerveDriveKinematics
        //     new PIDConstants(.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        //     new PIDConstants(0.6, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        //     driveTrain::setModuleStatesFromTrajectory, // Module states consumer used to output to the drive subsystem
        //     eventMap,
        //     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        //     driveTrain // The drive subsystem. Used to properly set the requirements of path following commands
        // );
        // driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING);
        
        Command fullAuto = generateFullAuto(wireCover, driveTrain);
        
            
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
            new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
            new WaitCommand(1.2),
            new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
            new WaitCommand(1),
            new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
            new WaitCommand(1),
            fullAuto.alongWith(new WaitCommand(5)),
            new InstantCommand(() -> driveTrain.setWantedState(Drivetrain.WantedState.AUTO_BALANCE))
            );
    }

    private static Command generateFullAuto(List<PathPlannerTrajectory> path, Drivetrain driveTrain){
            // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        
        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveTrain::getPose, // Pose2d supplier// Pose2d consumer, used to reset odometry at the beginning of auto
            driveTrain::resetPose,
            driveTrain.getKinematics(), // SwerveDriveKinematics
            new PIDConstants(.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.6, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            driveTrain::setModuleStatesFromTrajectory, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveTrain // The drive subsystem. Used to properly set the requirements of path following commands
        );
        
        Command fullAuto = autoBuilder.fullAuto(path);

        return fullAuto;
    }

}