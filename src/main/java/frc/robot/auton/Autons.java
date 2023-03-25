package frc.robot.auton;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.commands.ArmWantedStateCommand;
import frc.robot.auton.commands.IntakeWantedStateCommand;
import frc.robot.auton.commands.TheoryPath;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.SystemState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Autons {
    
    private static List<PathPlannerTrajectory> center = PathPlanner.loadPathGroup("New center", new PathConstraints(2.5, 2),
     new PathConstraints(2.5, 1.5),
      new PathConstraints(2.5, 2),
     new PathConstraints(2.5, 1.5));

    private static List<PathPlannerTrajectory> clearBlue = PathPlanner.loadPathGroup("New Clear", new PathConstraints(2.5, 1.75), new PathConstraints(2.5,1.75), new PathConstraints(2.5,1.75));

    private static List<PathPlannerTrajectory> clearRed = PathPlanner.loadPathGroup("New Clear2", new PathConstraints(2.5, 1.75), new PathConstraints(2.5,1.75), new PathConstraints(2.5,1.75));

    private static List<PathPlannerTrajectory> wireCover = PathPlanner.loadPathGroup("WireCover", new PathConstraints(2.5, 3));
    
    public static Command center(Drivetrain driveTrain, Arm arm, Intake intake) {

        Command[] fullAuto = TheoryPath.getPathLegs(center, driveTrain);

    
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(1.2),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto[0],
                fullAuto[1],
                new ParallelCommandGroup(fullAuto[2], new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE), new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE)),
                new ParallelCommandGroup(fullAuto[3], new ArmWantedStateCommand(arm, SystemState.NEUTRAL)),
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new InstantCommand(() -> driveTrain.setWantedState(Drivetrain.WantedState.AUTO_BALANCE))
            );
    }

    public static Command emergencyDonNothing(Drivetrain drivetrain){
        return new InstantCommand(() -> drivetrain.drive(0,0,0,true));
    }

    public static Command clearBlue(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearBlue, driveTrain);
        
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(1.2),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                new ParallelCommandGroup(fullAuto[0], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL))),
                fullAuto[1],
                new ArmWantedStateCommand(arm, SystemState.AUTON_HIGH),
                new WaitCommand(1),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                fullAuto[2]
                // new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE)
            );
    }

    public static Command clearRed(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearRed, driveTrain);
        
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(1.2),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                new ParallelCommandGroup(fullAuto[0], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL))),
                fullAuto[1],
                new ArmWantedStateCommand(arm, SystemState.AUTON_HIGH),
                new WaitCommand(1),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                fullAuto[2]
                // new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE)
            );
    }

    public static Command wireCover(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(wireCover, driveTrain);
        
            
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
            new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
            new WaitCommand(1.2),
            new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
            new WaitCommand(1),
            new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
            new WaitCommand(1),
            fullAuto[0]
            );
    }

}