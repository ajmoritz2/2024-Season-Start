package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight implements Subsystem {

    private enum SystemState{
        NEUTRAL,
        CONE,
        CUBE
    }

    SlewRateLimiter rotationSlew = new SlewRateLimiter(1, -1, 0);

    public static NetworkTable Limelight;
    public static NetworkTableEntry V;
    public static NetworkTableEntry X;
    public static NetworkTableEntry Y;
    public static NetworkTableEntry A;

    private SystemState currentState = SystemState.NEUTRAL;
    private SystemState wantedState = SystemState.NEUTRAL;

    private final XboxController controller;

    public Limelight(XboxController controller){
        this.controller = controller;

        Limelight = NetworkTableInstance.getDefault().getTable("limelight");
        X = Limelight.getEntry("tx");
        Y = Limelight.getEntry("ty");
        A = Limelight.getEntry("ta");
        V = Limelight.getEntry("tv");
    }

    @Override
    public void processLoop(double timestamp) {
        // TODO Auto-generated method stub
        SystemState newState;
        switch(currentState){
            default:
            case NEUTRAL:
                newState = stateChange();
                break;
            case CONE:
                newState = stateChange();
                break;
            case CUBE:
                newState = stateChange();
                break;
        }
        if (wantedState != currentState)
            currentState = newState;
    }

    @Override
    public void readPeriodicInputs(double timestamp){
        
        if(controller.getRightBumperPressed())
            setWantedState(SystemState.CONE);
        if(controller.getRightBumperReleased())
            setWantedState(SystemState.NEUTRAL);

        // if (Robot.m_robotContainer.drivetrain.getRightTrigger()){
        //     setWantedState(SystemState.CUBE);
        // } else if (currentState == SystemState.CUBE){
        //     setWantedState(SystemState.NEUTRAL);
        // }
        
    }

    @Override
    public void writePeriodicOutputs(double timestamp){
        switch(currentState){
            case CONE:
                setCameraMode(0, 0 , 2);
                break;
            case CUBE:
                setCameraMode(0, 0 , 1);
                break;
            default:
            case NEUTRAL:
                setCameraMode(1, 1 , 2);
                break;
        }
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void outputTelemetry(double timestamp){
        SmartDashboard.putBoolean("Has target", getTarget());
        SmartDashboard.putNumber("getX", getXValue());
        SmartDashboard.putNumber("getY", getYValue());
        SmartDashboard.putNumber("Steering", steeringAdjust());
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    private SystemState stateChange(){
        return wantedState;
    }

    public void setWantedState(SystemState wanted){
        wantedState = wanted;
    }

    public double steeringAdjust() {

        double steeringAdjust = 0;
        final double heading_error = -X.getDouble(0.0);
        final double area = A.getDouble(0.0);

        final double Kp = 0.03;
        final double min_command = 0.03;
    
        if(Math.abs(heading_error)>(4.0)){
            if(heading_error>0)

              steeringAdjust = Kp*heading_error+min_command;
            else
              steeringAdjust = Kp*heading_error-min_command;
        }
            return rotationSlew.calculate(steeringAdjust);
      }

    public void setCameraMode(final double ledMode, final double camMode, final double pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }

    public boolean getTarget(){
   
        final double v = V.getDouble(0.0);
        boolean isThereTarget;

        if (v == 1) 
          isThereTarget = true;
        else
        isThereTarget = false;
    
        return isThereTarget;
      }
    
      public double getXValue() {
        final double x = X.getDouble(0.0);
        return x;
      }
    
      public double getYValue() {
        final double y = Y.getDouble(0.0);
        return y;
      }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public String getId() {
        // TODO Auto-generated method stub
        return null;
    }
    
}