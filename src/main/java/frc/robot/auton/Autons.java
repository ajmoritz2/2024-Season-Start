
package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.SystemState;
import frc.robot.subsystems.Intake.WantedState;
import frc.robot.auton.commands.*;

public class Autons {
 private static final PathPlannerTrajectory center1Part1 = PathPlanner.loadPath("Center1Part1",  Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
 private static final PathPlannerTrajectory center1Part2 = PathPlanner.loadPath("Center1Part2", 2, 1);
 private static final PathPlannerTrajectory test = PathPlanner.loadPath("4.4m", Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);


public static Command test(Drivetrain driveTrain, Arm arm, Intake intake){
    final PathPlannerCommand part1 = new PathPlannerCommand(center1Part1, driveTrain,true);
    final PathPlannerCommand part2 = new PathPlannerCommand(center1Part2, driveTrain, false);
    final PathPlannerCommand testRun = new PathPlannerCommand(test, driveTrain,true);
    return new SequentialCommandGroup(
        new InstantCommand(() -> driveTrain.drive(0.0, 0.0, 0.0, true)),
        part1.alongWith(new WaitCommand(1.4)),
        part2.alongWith(new WaitCommand(3))


        /* 
        new WaitCommand(4),
        new ParallelCommandGroup(
        new ArmWantedStateCommand(arm, SystemState.MID),
        new IntakeWantedStateCommand(intake, WantedState.IDLE)
    */
        
    );
}

public static Command curveTest(Drivetrain drivetrain){
    
    return new SequentialCommandGroup(
        
    );
}

}