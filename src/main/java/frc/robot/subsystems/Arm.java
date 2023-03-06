package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Arm implements Subsystem {

    private TalonFX liftMotor;
   // private TalonFX rotateMotorLeft;
    private TalonFX rotateMotorRight;
    private DigitalInput armLimitSwitch;
    private DigitalInput rotateLimitSwitch;
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

    private final Intake intake;

    private final PS4Controller controller;

    public Arm(PS4Controller controller, Intake intake){

        this.intake = intake;
        this.controller = controller;

        liftMotor = new TalonFX(Constants.Arm.EXTENDMOTOR, "MANIPbus");
        rotateMotorRight = new TalonFX(Constants.Arm.ROTATEMOTOR2, "MANIPbus");
        //rotateMotorLeft = new TalonFX(Constants.Arm.ROTATEMOTOR1, "MANIPbus");
        armEncoder = new CANCoder(Constants.Arm.EXTENDENCODER, "MANIPbus");
        rotateEncoder = new CANCoder(Constants.Arm.ROTATEENCODER, "MANIPbus");
        liftMotor.configFactoryDefault();
        //rotateMotorLeft.configFactoryDefault();
        rotateMotorRight.configFactoryDefault();
        

        armLimitSwitch = new DigitalInput(1);
        rotateLimitSwitch = new DigitalInput(0);

        liftMotor.setNeutralMode(NeutralMode.Brake);
        //rotateMotorLeft.setNeutralMode(NeutralMode.Brake);
        rotateMotorRight.setNeutralMode(NeutralMode.Brake);

        liftMotor.selectProfileSlot(0, 0);
		liftMotor.config_kF(0, 0.125);
		liftMotor.config_kP(0,2); //0.1
		liftMotor.config_kI(0, 0);
		liftMotor.config_kD(0, 0);

        liftMotor.configPeakOutputForward(0.5);
        liftMotor.configPeakOutputReverse(-0.5);
        /* 
        rotateMotorLeft.selectProfileSlot(0, 0);
		rotateMotorLeft.config_kF(0, 0.125);
		rotateMotorLeft.config_kP(0,2.4); //2.4 5% overshoot
		rotateMotorLeft.config_kI(0, 0);
		rotateMotorLeft.config_kD(0, 0);
        rotateMotorLeft.configPeakOutputForward(0.4);
        rotateMotorLeft.configPeakOutputReverse(-0.4);
        */
        rotateMotorRight.selectProfileSlot(0, 0);
		rotateMotorRight.config_kF(0, 0.125);
		rotateMotorRight.config_kP(0,2.4); //2.4 5% overshoot
		rotateMotorRight.config_kI(0, 0);
		rotateMotorRight.config_kD(0, 0);

        rotateMotorRight.configPeakOutputForward(0.5);
        rotateMotorRight.configPeakOutputReverse(-0.5);

        feedforward = new ElevatorFeedforward(0.01, 0, 0.06);
        liftMotor.setSensorPhase(true);
       // rotateMotorLeft.setSensorPhase(true);
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
             case MID:
                 newState = handleManual();
                 break;
             case HIGH:
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
        if(!armLimitSwitch.get())
            zeroArmSensors();

        if(!rotateLimitSwitch.get())
            zeroRotateSensors();

        
        if(intake.getIntakeCurrent()>=200 && intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.PLACING && intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.IDLE)
            setWantedState(SystemState.NEUTRAL);

        if(controller.getCrossButtonPressed())
            setWantedState(SystemState.GROUND_ANGLE);
        if(controller.getCrossButtonReleased())
            setWantedState(SystemState.NEUTRAL);

        if(controller.getCircleButtonPressed())
            setWantedState(SystemState.MID);

        if(controller.getTriangleButtonPressed())
            setWantedState(SystemState.NEUTRAL);

        if(controller.getSquareButtonPressed())
            setWantedState(SystemState.HIGH);

    }

    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        switch (currentState){
            case GROUND_ANGLE:
                configRotate(-86190); //target -88190
                configExtend(0);
                break;
             case MID:
                configRotate(-46080); //target -46080
                configExtend(39949); //target 39949
                break;
            case HIGH:
                configRotate(-43320); //target-45320
                configExtend(116256); //traget 116256
                break;
            default:
            case NEUTRAL:
                configRotate(0);
                configExtend(0);
                break;
            
        }
    }

    @Override
    public void stop() {
        
    }

    @Override
    public void outputTelemetry(double timestamp){
        double calced = feedforward.calculate(m_setpoint.position);

        SmartDashboard.putBoolean("ArmLimitSwitch", armLimitSwitch.get());
        SmartDashboard.putBoolean("RotateLimitSwitch", rotateLimitSwitch.get());
        SmartDashboard.putNumber("Current Lift Pos", liftMotor.getSelectedSensorPosition());
        //SmartDashboard.putNumber("Current Rot Pos", rotateMotorLeft.getSelectedSensorPosition());
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
        //rotateMotorLeft.set(ControlMode.Position, pos);
        rotateMotorRight.set(ControlMode.Position, pos);
    }
    
    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
        liftMotor.setSelectedSensorPosition(0);
        //rotateMotorLeft.setSelectedSensorPosition(0);
        rotateMotorRight.setSelectedSensorPosition(0);
       // armEncoder.setPosition(0);
       // rotateEncoder.setPosition(0);
    }
   
    public void zeroRotateSensors(){
        //rotateMotorLeft.setSelectedSensorPosition(0);
        rotateMotorRight.setSelectedSensorPosition(0);
    }
    public void zeroArmSensors(){
        liftMotor.setSelectedSensorPosition(0);
    }

    

    @Override
    public String getId() {
        return null;
    }

    private float ensureRange(float value, float min, float max) {
        return Math.min(Math.max(value, min), max);
     }
    
}