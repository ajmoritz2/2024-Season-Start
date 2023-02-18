package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Arm implements Subsystem {

    private TalonFX liftMotor;
    private TalonFX rotateMotorLeft;
    private TalonFX rotateMotorRight;
    private CANCoder armEncoder;
    private CANCoder rotateEncoder;
    private ElevatorFeedforward feedforward;

    public enum SystemState{
        NEUTRAL,
        GROUND_ANGLE,
        HUMAN_FOLD,
        PLACING,
        HIGH,
        MID,
        TRAVEL,
        START;
    }

    private SystemState currentState = SystemState.NEUTRAL;
    private SystemState wantedState = SystemState.NEUTRAL;

    private final TrapezoidProfile.Constraints m_constraints =

    new TrapezoidProfile.Constraints(10, 2);

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private final XboxController controller;

    public Arm(XboxController controller){

        this.controller = controller;

        liftMotor = new TalonFX(Constants.Arm.EXTENDMOTOR);
        rotateMotorRight = new TalonFX(Constants.Arm.ROTATEMOTOR2);
        rotateMotorLeft = new TalonFX(Constants.Arm.ROTATEMOTOR1);
        armEncoder = new CANCoder(Constants.Arm.EXTENDENCODER);
        rotateEncoder = new CANCoder(Constants.Arm.ROTATEENCODER);
        liftMotor.configFactoryDefault();
        rotateMotorLeft.configFactoryDefault();

        liftMotor.setNeutralMode(NeutralMode.Brake);
        rotateMotorLeft.setNeutralMode(NeutralMode.Brake);
        rotateMotorRight.setNeutralMode(NeutralMode.Brake);

        liftMotor.selectProfileSlot(0, 0);
		liftMotor.config_kF(0, 0.125);
		liftMotor.config_kP(0,0.1); //0.0625
		liftMotor.config_kI(0, 0);
		liftMotor.config_kD(0, 0);

        liftMotor.configPeakOutputForward(0.2);
        liftMotor.configPeakOutputReverse(0.2);


        rotateMotorLeft.selectProfileSlot(0, 0);
		rotateMotorLeft.config_kF(0, 0.125);
		rotateMotorLeft.config_kP(0,0.0625);
		rotateMotorLeft.config_kI(0, 0);
		rotateMotorLeft.config_kD(0, 0);

        rotateMotorLeft.configPeakOutputForward(0.2);
        rotateMotorLeft.configPeakOutputReverse(0.2);

        rotateMotorRight.selectProfileSlot(0, 0);
		rotateMotorRight.config_kF(0, 0.125);
		rotateMotorRight.config_kP(0,0.0625);
		rotateMotorRight.config_kI(0, 0);
		rotateMotorRight.config_kD(0, 0);

        rotateMotorRight.configPeakOutputForward(0.2);
        rotateMotorRight.configPeakOutputReverse(0.2);

        feedforward = new ElevatorFeedforward(0.01, 0, 0.06);
        liftMotor.setSensorPhase(true);
        rotateMotorLeft.setSensorPhase(true);
        rotateMotorRight.setSensorPhase(true);

        //rotateMotorLeft.follow(rotateMotorRight);
        // liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10,15,0.5));
    }

    @Override
    public void processLoop(double timestamp) {
        
         SystemState newState;
         switch(currentState){
             default:
             case NEUTRAL:
                 newState = handleManual();
                 break;
             case GROUND_ANGLE:
                 newState = handleManual();
                 break;
             case HUMAN_FOLD:
                 newState = handleManual();
                 break;
             case PLACING:
                 newState = handleManual();
                 break;
         }

        if (wantedState != currentState) {
			currentState = newState;
        }
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        // Yes I am going to use short hand if statements because it looks better.
        // (condition) ? true : false;

        // I also do != Neutral because that should make sure it isn't in some weird mode and have some motors clash
        // Examples of controller inputs:

    //    if (controller.getAButtonReleased())
    //         wantedState = (currentState != SystemState.NEUTRAL) ? SystemState.NEUTRAL : SystemState.EXTEND; 
       

    //    if (controller.getBButtonReleased())
    //     wantedState = (currentState != SystemState.NEUTRAL) ? SystemState.NEUTRAL : SystemState.RETRACT;  
       if(controller.getAButtonPressed())
            wantedState = (currentState != SystemState.NEUTRAL) ? SystemState.NEUTRAL : SystemState.GROUND_ANGLE;

       else if(controller.getBButtonPressed())
            setWantedState(SystemState.NEUTRAL);

    }

    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        switch (currentState){
            case GROUND_ANGLE:
                configRotate(50000);
                break;
            //case HUMAN_FOLD:
              //  configRotate(-100000);
              //  break;
            // case PLACING:
              //  configExtend(0);
              //  break;
            default:
            case NEUTRAL:
                configRotate(-100000);
                break;
            //case HIGH:
                //TODO: put something here
               // break;
           // case MID:
                // MID
                //break;
        }
    }

    @Override
    public void stop() {
        
    }

    @Override
    public void outputTelemetry(double timestamp){
        double calced = feedforward.calculate(m_setpoint.position);


        SmartDashboard.putNumber("Current Lift Pos", liftMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Current Rot Pos", rotateMotorLeft.getSelectedSensorPosition());
    }

    private SystemState handleManual(){
        return wantedState;
    }

    public void setWantedState(SystemState wanted){
        wantedState = wanted;
    }

    public void configExtend(double position){
        liftMotor.set(ControlMode.Position, position);
    }
   
    public void configRotate(double pos){
        rotateMotorLeft.set(ControlMode.Position, pos);
        rotateMotorRight.set(ControlMode.Position, pos);
    }
    
    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
        liftMotor.setSelectedSensorPosition(0);
        rotateMotorLeft.setSelectedSensorPosition(0);
        rotateMotorRight.setSelectedSensorPosition(0);
       // armEncoder.setPosition(0);
       // rotateEncoder.setPosition(0);
    }

    @Override
    public String getId() {
        return null;
    }

    private float ensureRange(float value, float min, float max) {
        return Math.min(Math.max(value, min), max);
     }
    
}
