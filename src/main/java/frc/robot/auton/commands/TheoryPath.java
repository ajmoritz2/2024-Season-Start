package frc.robot.auton.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auton.commands.PathPlannerCommand;
import frc.robot.subsystems.Drivetrain;

public class TheoryPath extends CommandBase{

    public static PathPlannerCommand[] getPathLegs(List<PathPlannerTrajectory> paths, Drivetrain drivetrain){
        List<PathPlannerCommand> legs = new ArrayList<PathPlannerCommand>();
        int i = 0;
        
        for (PathPlannerTrajectory path : paths){
            if (i == 0){
                legs.add(new PathPlannerCommand(path, drivetrain, true));
                i++;
            } else 
                legs.add(new PathPlannerCommand(path, drivetrain, false));
            

        }

        return legs.toArray(PathPlannerCommand[]::new);
    }

    

}
