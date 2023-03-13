package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        ZERO,
        PLACING,
        HIGH,
        MID,
        TRAVEL,
        AUTON_MID,
        AUTON_HIGH,
        MANUAL
    }

    private SystemState currentState = SystemState.NEUTRAL;
    private SystemState wantedState = SystemState.NEUTRAL;

    private final TrapezoidProfile.Constraints m_constraints =

    new TrapezoidProfile.Constraints(10, 2);

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private final Intake intake;

    private final PS4Controller controller;

    private boolean manualMode = false;

    public Arm(PS4Controller controller, Intake intake){

        this.intake = intake;
        this.controller = controller;

        liftMotor = new TalonFX(Constants.ARM.EXTENDMOTOR, "MANIPbus");
        rotateMotorRight = new TalonFX(Constants.ARM.ROTATEMOTOR2, "MANIPbus");
        //rotateMotorLeft = new TalonFX(Constants.Arm.ROTATEMOTOR1, "MANIPbus");
        armEncoder = new CANCoder(Constants.ARM.EXTENDENCODER, "MANIPbus");
        rotateEncoder = new CANCoder(Constants.ARM.ROTATEENCODER, "MANIPbus");
        liftMotor.configFactoryDefault();
        //rotateMotorLeft.configFactoryDefault();
        rotateMotorRight.configFactoryDefault();
        

        armLimitSwitch = new DigitalInput(9);
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

        rotateMotorRight.configPeakOutputForward(0.3);
        rotateMotorRight.configPeakOutputReverse(-0.3);

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
             case ZERO:
                newState = handleManual();
                break;
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
            case AUTON_MID:
                newState = handleManual();
                break;
            case AUTON_HIGH:
                newState = handleManual();
                break;
            case  MANUAL:
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
            rotateMotorRight.setSelectedSensorPosition(5174);
            // zeroRotateSensors();

        if (!(currentState == SystemState.MANUAL)){
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

            if(controller.getR1ButtonPressed())
                setWantedState(SystemState.AUTON_HIGH); // hUMAN fOLD
            if(controller.getR1ButtonReleased())
                setWantedState(SystemState.NEUTRAL);
            if (controller.getOptionsButtonPressed())
                setWantedState(SystemState.ZERO);
            
        } else {
            if (controller.getOptionsButtonPressed())
                rotateMotorRight.setSelectedSensorPosition(0);
        }

        if (controller.getPSButtonPressed()){
            
            if (currentState == SystemState.MANUAL){
                setWantedState(SystemState.NEUTRAL);
                manualMode = false;
            }else{
                setWantedState(SystemState.MANUAL);
                manualMode= true;
            }
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        switch (currentState){
            case GROUND_ANGLE:
                configRotate(-85190); //target -88190
                configExtend(0);
                break;
             case MID:
                configRotate(-42080); //ta  rget -46080
                configExtend(39949); //target 39949
                break;
            case HIGH:
                configRotate(-39320); //target-45320
                configExtend(116256); //traget 116256
                break;
            case AUTON_MID:
                configRotate(46080);
                configExtend(39949);
                break;
            case AUTON_HIGH:
                configRotate(41320-4000);
                configExtend(112256);
                break;
            case MANUAL:
                manualControl(controller.getLeftX(), -controller.getRightY());
                break;
            case HUMAN_FOLD:
                configRotate(-41320);
                configExtend(0);
                break;
            case ZERO:
                configRotate(70057);
                configExtend(0);
                break;
            default:
            case NEUTRAL:
                // neutralize();
                configRotate(0);
                configExtend(0);
                break;
            
        }
    }

    public void neutralize(){
        if (rotateLimitSwitch.get()){
            if (rotateMotorRight.getSelectedSensorPosition() < 0){
                rotateMotorRight.set(ControlMode.PercentOutput, 0.4);
            } else {
                rotateMotorRight.set(ControlMode.PercentOutput, -0.4); 
            }
        } else {
            rotateMotorRight.set(ControlMode.PercentOutput, 0);
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
        SmartDashboard.putNumber("Current Rot Pos", rotateMotorRight.getSelectedSensorPosition());
        SmartDashboard.putBoolean("Manual Mode", manualMode);
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
        // liftMotor.setSelectedSensorPosition(0);
        // //rotateMotorLeft.setSelectedSensorPosition(0);
        // rotateMotorRight.setSelectedSensorPosition(0);
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

    public void manualControl(double rotateOutput, double armOutput){
        rotateMotorRight.set(ControlMode.PercentOutput, rotateOutput/4);
        if (!armLimitSwitch.get() && armOutput < 0){
            liftMotor.set(ControlMode.PercentOutput, 0);
        } else {
            liftMotor.set(ControlMode.PercentOutput, armOutput);
        }
        
    }

    

    @Override
    public String getId() {
        return null;
    }

    private float ensureRange(float value, float min, float max) {
        return Math.min(Math.max(value, min), max);
     }
    
}