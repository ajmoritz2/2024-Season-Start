package frc.robot.auton.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.SwerveDrivetrain;


public class PathPlannerCommand extends SwerveControllerCommand {
  /** Creates a new PathPlannerSwerveTrajectoryCommand. */
  private PathPlannerTrajectory trajectory;
  private Drivetrain drivetrain;
  private final boolean resetOdometry;

    //TODO: relocate to constants file
    private static final double translateKp = 12;
    private static final double translateKi = 0;
    private static final double translateKd = 0;
    private static final double rotateKp = 1; //.3
    private static final double rotateKi = 0;
    private static final double rotateKd = 0;

  private static ProfiledPIDController thetaController =  new ProfiledPIDController(rotateKp, rotateKi, rotateKd, Constants.AutoConstants.kThetaControllerConstraints);

  public PathPlannerCommand(PathPlannerTrajectory trajectory, Drivetrain drivetrain){
      this(trajectory, drivetrain, true);
  }
  
  public PathPlannerCommand(PathPlannerTrajectory trajectory, Drivetrain drivetrain, boolean resetOdometry) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    super(trajectory,
        drivetrain::getPose,
        drivetrain.getKinematics(),
        new PIDController(translateKp, translateKi, translateKd),
        new PIDController(translateKp, translateKi, translateKd),
        thetaController,
        drivetrain::setModuleStatesFromTrajectory, 
        drivetrain);
    
      this.trajectory = trajectory;
      this.drivetrain = drivetrain;
      this.resetOdometry = resetOdometry;
      thetaController.enableContinuousInput(-Math.PI, Math.PI); //TODO check if pigeon needs this
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    drivetrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING);

    if(resetOdometry){
        drivetrain.initAutonPosition(trajectory.getInitialState());
        SmartDashboard.putString("auton/pose",trajectory.getInitialPose().toString());
        SmartDashboard.putString("auton/angle",trajectory.getInitialState().holonomicRotation.toString());    
    }
  }

  @Override
  public void end(boolean interrupted){
    super.end(interrupted);
    this.drivetrain.setWantedState(Drivetrain.WantedState.IDLE);
  }

  /*@Override
  public boolean isFinished() {
    return true;
  }*/
}