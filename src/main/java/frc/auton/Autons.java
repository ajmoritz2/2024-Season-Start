
package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.commands.*;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Autons {
  

public static Command test(Drivetrain driveTrain){
    
    return new SequentialCommandGroup(
    
    );
}

public static Command curveTest(Drivetrain drivetrain){
    
    return new SequentialCommandGroup(
        
    );
}

}