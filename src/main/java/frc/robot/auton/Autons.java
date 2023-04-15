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
    
    private static List<PathPlannerTrajectory> center = PathPlanner.loadPathGroup("CenterReal", new PathConstraints(2.5, 2),
     new PathConstraints(2.5, 1.5),
      new PathConstraints(2.5, 2),
     new PathConstraints(2.5, 1.5));

     private static List<PathPlannerTrajectory> centernotouch = PathPlanner.loadPathGroup("CenterReal", new PathConstraints(2.5, 2),
     new PathConstraints(2.5, 1.5),
      new PathConstraints(2.5, 2),
     new PathConstraints(2.5, 1.5));

     private static List<PathPlannerTrajectory> lowCenterBlue = PathPlanner.loadPathGroup("Low Center Blue", new PathConstraints(2.5, 2),
     new PathConstraints(2.5, 1.5),
      new PathConstraints(2.5, 2),
     new PathConstraints(2.5, 1.5));

    private static List<PathPlannerTrajectory> clearBlue = PathPlanner.loadPathGroup("Clear Blue", new PathConstraints(2.5, 1.75), new PathConstraints(2.5,1.75), new PathConstraints(2.5,1.75));

    private static List<PathPlannerTrajectory> clearRed = PathPlanner.loadPathGroup("Clear Red", new PathConstraints(2.5, 1.75), new PathConstraints(2.5,1.75), new PathConstraints(2.5,1.75));

    private static List<PathPlannerTrajectory> wireCoverBlue = PathPlanner.loadPathGroup("WireCover Blue", new PathConstraints(2.5, 1.75), new PathConstraints(2.5, 1.75), new PathConstraints(2.5, 1.75));

    private static List<PathPlannerTrajectory> wireCoverRed = PathPlanner.loadPathGroup("WireCover Red", new PathConstraints(2, 1.75), new PathConstraints(2, 1.75), new PathConstraints(2, 1.75));
   
    private static List<PathPlannerTrajectory> shortI = PathPlanner.loadPathGroup("short", new PathConstraints(1.75, 0.1));


    public static Command emergencyDonNothing(Drivetrain drivetrain){
        return new InstantCommand(() -> drivetrain.drive(0,0,0,true));
    }

    private static double firstWait = 1.7;

    public static Command shortOne(Drivetrain drivetrain){
        Command[] fullAuto = TheoryPath.getPathLegs(shortI, drivetrain);
        return new SequentialCommandGroup(fullAuto[0]);
    }

    public static Command center1(Drivetrain driveTrain, Arm arm, Intake intake) {

        Command[] fullAuto = TheoryPath.getPathLegs(center, driveTrain);

    
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE))
            );
    }

    public static Command center2(Drivetrain driveTrain, Arm arm, Intake intake) {

        Command[] fullAuto = TheoryPath.getPathLegs(center, driveTrain);

    
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto[0],
                fullAuto[1],
                fullAuto[2]
            );
    }

    public static Command center3(Drivetrain driveTrain, Arm arm, Intake intake) {

        Command[] fullAuto = TheoryPath.getPathLegs(center, driveTrain);

    
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto[0],
                fullAuto[1],
                fullAuto[2],
                fullAuto[3], 
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new InstantCommand(() -> driveTrain.setWantedState(Drivetrain.WantedState.AUTO_BALANCE))
            );
    }

    public static Command center4(Drivetrain driveTrain, Arm arm, Intake intake) {

        Command[] fullAuto = TheoryPath.getPathLegs(center, driveTrain);

    
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
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

    public static Command center5(Drivetrain driveTrain, Arm arm, Intake intake) {

        Command[] fullAuto = TheoryPath.getPathLegs(center, driveTrain);

    
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto[0],
                fullAuto[1],
                new ParallelCommandGroup(fullAuto[2], new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE), new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE)),
                new ParallelCommandGroup(fullAuto[3], new ArmWantedStateCommand(arm, SystemState.MID)),
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ParallelCommandGroup(new InstantCommand(() -> driveTrain.setWantedState(Drivetrain.WantedState.AUTO_BALANCE)), 
                new SequentialCommandGroup(new WaitCommand(1.75),
                new IntakeWantedStateCommand(intake, Intake.WantedState.SHOOT)),
                new SequentialCommandGroup(new WaitCommand(1.0), 
                new ArmWantedStateCommand(arm, Arm.SystemState.PUNCH)))
            );
    }

    public static Command centerNoTouch(Drivetrain driveTrain, Arm arm, Intake intake) {

        Command[] fullAuto = TheoryPath.getPathLegs(centernotouch, driveTrain);

    
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto[0],
                fullAuto[1],
                fullAuto[2],
                fullAuto[3], 
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new InstantCommand(() -> driveTrain.setWantedState(Drivetrain.WantedState.AUTO_BALANCE))
            );
    }

    public static Command centerLow(Drivetrain driveTrain, Arm arm, Intake intake) {

        Command[] fullAuto = TheoryPath.getPathLegs(lowCenterBlue, driveTrain);

    
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
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
                new ParallelCommandGroup(new InstantCommand(() -> driveTrain.setWantedState(Drivetrain.WantedState.AUTO_BALANCE)), new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.SHOOT),new WaitCommand(1), new IntakeWantedStateCommand(intake, Intake.WantedState.PLACING)))
            );
    }

    public static Command clearBlue1(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearBlue, driveTrain);
          
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL)     
            );
    }

    public static Command clearBlue2(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearBlue, driveTrain);
          
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto[0]      
            );
    }
    
    public static Command clearBlue3(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearBlue, driveTrain);
          
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
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
                new ParallelCommandGroup(fullAuto[2], new SequentialCommandGroup(new WaitCommand(2), new InstantCommand(() -> driveTrain.drive(0, 0, 0, true))))
            );
    }

    public static Command clearBlue4(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearBlue, driveTrain);
          
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(.7),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(.7),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                new ParallelCommandGroup(fullAuto[0], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL))),
                fullAuto[1],
                new ArmWantedStateCommand(arm, SystemState.AUTON_HIGH),
                new WaitCommand(1),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(.5),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new WaitCommand(.2),
                new ParallelCommandGroup(fullAuto[2], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL)))
            );
    }

    public static Command clearRed1(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearRed, driveTrain);
          
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL)     
            );
     }

    public static Command clearRed2(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearRed, driveTrain);
          
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto[0]      
            );
     }

    public static Command clearRed3(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearRed, driveTrain);
          
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
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
                new ParallelCommandGroup(fullAuto[2], new SequentialCommandGroup(new WaitCommand(2),new InstantCommand(() -> driveTrain.drive(0, 0, 0, true))))
            );
     }

    public static Command clearRed4(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(clearRed, driveTrain);
          
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
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
                new ParallelCommandGroup(fullAuto[2], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL)))
            );
     }
 
    public static Command wireCoverBlue1(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(wireCoverBlue, driveTrain);
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL)     
            );
    }

    public static Command wireCoverBlue2(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(wireCoverBlue, driveTrain);
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto[0]      
            );
    }

    public static Command wireCoverBlue3(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(wireCoverBlue, driveTrain);
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                new ParallelCommandGroup(fullAuto[0], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL))),
                fullAuto[1],
                new ArmWantedStateCommand(arm, SystemState.AUTON_HIGH),
                new WaitCommand(1),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new ParallelCommandGroup(fullAuto[2], new SequentialCommandGroup(new WaitCommand(2),new InstantCommand(() -> driveTrain.drive(0, 0, 0, true))))
            );
    }

    public static Command wireCoverBlue4(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(wireCoverBlue, driveTrain);
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                new ParallelCommandGroup(fullAuto[0], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL))),
                fullAuto[1],
                new ArmWantedStateCommand(arm, SystemState.AUTON_HIGH),
                new WaitCommand(1),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new ParallelCommandGroup(fullAuto[2], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL)))
            );
    }

    public static Command wireCoverRed1(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(wireCoverRed, driveTrain);
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL)     
            );
    }

    public static Command wireCoverRed2(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(wireCoverRed, driveTrain);
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                fullAuto[0]      
            );
    }

    public static Command wireCoverRed3(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(wireCoverRed, driveTrain);
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                new ParallelCommandGroup(fullAuto[0], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL))),
                fullAuto[1],
                new ArmWantedStateCommand(arm, SystemState.AUTON_HIGH),
                new WaitCommand(1),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new ParallelCommandGroup(fullAuto[2], new SequentialCommandGroup(new WaitCommand(2),new InstantCommand(() -> driveTrain.drive(0, 0, 0, true))))
            );
    }

    public static Command wireCoverRed4(Drivetrain driveTrain, Arm arm, Intake intake){
        
        Command[] fullAuto = TheoryPath.getPathLegs(wireCoverRed, driveTrain);
        
            
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
                new ArmWantedStateCommand(arm,SystemState.AUTON_HIGH),
                new WaitCommand(firstWait),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ArmWantedStateCommand(arm, SystemState.NEUTRAL),
                new WaitCommand(1),
                new InstantCommand(()-> driveTrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING)),
                new ParallelCommandGroup(fullAuto[0], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL))),
                fullAuto[1],
                new ArmWantedStateCommand(arm, SystemState.AUTON_HIGH),
                new WaitCommand(1),
                new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.PLACING),
                new WaitCommand(1),
                new ParallelCommandGroup(new ArmWantedStateCommand(arm, SystemState.NEUTRAL), new IntakeWantedStateCommand(intake, frc.robot.subsystems.Intake.WantedState.IDLE)),
                new ParallelCommandGroup(fullAuto[2], new SequentialCommandGroup(new ArmWantedStateCommand(arm, SystemState.GROUND_ANGLE),new IntakeWantedStateCommand(intake, Intake.WantedState.INTAKING_CUBE),new WaitCommand(3.5),new ArmWantedStateCommand(arm, SystemState.NEUTRAL)))
            );
    }

}  

