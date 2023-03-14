package frc.robot.subsystems;


import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;

import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
// import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Arm implements Subsystem {

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

    private SystemState m_currentState = SystemState.NEUTRAL;
    private SystemState m_wantedState = SystemState.NEUTRAL;

    protected TalonFX m_extendMotor;
    protected MotionMagicVoltage m_extendMotorMMV;
    // protected CANcoder m_extendEncoder;
    protected DigitalInput m_extendLimitSwitch;

    protected TalonFX m_rotateMotor;
    protected MotionMagicVoltage m_rotateMotorMMV;
    protected CANcoder m_rotateEncoder;
    protected DigitalInput m_rotateLimitSwitch;


    private final Intake m_intake;
    private final PS4Controller m_controller;

    private boolean m_manualMode = false;

    public Arm(PS4Controller controller, Intake intake){

        m_intake = intake;
        m_controller = controller;

        // extendEncoderInit();
		extendMotorInit();
        m_extendLimitSwitch = new DigitalInput(9);

        //---------------------------------------------------------------------
        rotateEncoderInit();
        rotateMotorInit();
		m_rotateLimitSwitch = new DigitalInput(0);
        //--------------------------------------------------------------------- 
        // feedforward = new ElevatorFeedforward(0.01, 0, 0.06);
        // liftMotor.setSensorPhase(true);
        // rotateMotorLeft.setSensorPhase(true);
        // rotateMotorRight.setSensorPhase(true);
    }

    private void extendMotorInit(){
        m_extendMotor = new TalonFX(Constants.ARM.EXTENDMOTOR, "MANIPbus");
        m_extendMotorMMV = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 20; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 40; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 50;   

		// from Phoenix v5 - kettering
		// liftMotor.config_kF(0, 0.125);
		// liftMotor.config_kP(0,2); //0.1
		// liftMotor.config_kI(0, 0);
		// liftMotor.config_kD(0, 0);
        // liftMotor.configPeakOutputForward(0.5);
		// liftMotor.configPeakOutputReverse(-0.5);
		// liftMotor.setSensorPhase(true);
		
        cfg.Slot0.kP = 4.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.0F; // Approximately 0.25V to get the mechanism moving

        // cfg.Feedback.SensorToMechanismRatio = 2F;
    
		//TODO make this like 0.5 percentoutput, so maxvoltage/2
        // cfg.Voltage.PeakForwardVoltage = 3.2; //3.2V is 20% of 16V
        // cfg.Voltage.PeakForwardVoltage = -3.2; //3.2V is 20% of 16V

        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit = 15.0;
        // m_extendMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10,15,0.5));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_extendMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure extend motor. Error: " + status.toString());
        }

        m_extendMotorMMV.OverrideBrakeDurNeutral = true;
        m_extendMotor.setRotorPosition(0);
        m_extendMotor.setVoltage(0);
    }

	private void rotateMotorInit(){
        m_rotateMotor = new TalonFX(Constants.ARM.ROTATEMOTOR, "MANIPbus");
        m_rotateMotorMMV = new MotionMagicVoltage(0);  

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 50;   

		// from Phoenix v5 - kettering
        // rotateMotorRight.selectProfileSlot(0, 0);
		// rotateMotorRight.config_kF(0, 0.125);
		// rotateMotorRight.config_kP(0,2.4); //2.4 5% overshoot
		// rotateMotorRight.config_kI(0, 0);
		// rotateMotorRight.config_kD(0, 0);
		// rotateMotorRight.configPeakOutputForward(0.3);
        // rotateMotorRight.configPeakOutputReverse(-0.3);
		// rotateMotorRight.setSensorPhase(true);
		
        cfg.Slot0.kP = 2.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving
    
        // cfg.Feedback.SensorToMechanismRatio = 2F;
        
        // cfg.Voltage.PeakForwardVoltage = 3.2; //3.2V is 20% of 16V
        // cfg.Voltage.PeakForwardVoltage = -3.2; //3.2V is 20% of 16V

        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit = 15.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_rotateMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure rotate right motor. Error: " + status.toString());
        }

        m_rotateMotorMMV.OverrideBrakeDurNeutral = true;
        m_rotateMotor.setRotorPosition(0);
        m_rotateMotor.setVoltage(0);
    }

    // private void extendEncoderInit(){
    //     m_extendEncoder = new CANcoder(Constants.ARM.EXTENDENCODER, "MANIPbus");

    //     /* Configure CANcoder */
    //     var cfg = new CANcoderConfiguration();

    //     /* User can change the configs if they want, or leave it empty for factory-default */

    //     m_extendEncoder.getConfigurator().apply(cfg);

    //     /* Speed up signals to an appropriate rate */
    //     m_extendEncoder.getPosition().setUpdateFrequency(100);
    //     m_extendEncoder.getVelocity().setUpdateFrequency(100);
    // }

    private void rotateEncoderInit(){
        m_rotateEncoder = new CANcoder(Constants.ARM.ROTATEENCODER, "MANIPbus");

        /* Configure CANcoder to zero the magnet appropriately */
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = 0.4;   //TODO
        m_rotateEncoder.getConfigurator().apply(cc_cfg);

        /* Speed up signals to an appropriate rate */
        m_rotateEncoder.getPosition().setUpdateFrequency(100);
        m_rotateEncoder.getVelocity().setUpdateFrequency(100);
    }
    @Override
    public void processLoop(double timestamp) {
        
         SystemState newState;
         switch(m_currentState){
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

        if (m_wantedState != m_currentState) {
			m_currentState = newState;
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

//TODO  - not sure need these anymore with encode on shaft
//        or at least zeroed differently now     
//        if(!extendLimitSwitch.get())
//            zeroArmSensors();
//
//        if(!rotateLimitSwitch.get())
//            rotateMotor.setSelectedSensorPosition(5174);
//            // zeroRotateSensors();

        if (!(m_currentState == SystemState.MANUAL)){
            if(m_intake.getIntakeCurrent()>=200 && m_intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.PLACING && m_intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.IDLE)
                setWantedState(SystemState.NEUTRAL);

            if(m_controller.getCrossButtonPressed())
                setWantedState(SystemState.GROUND_ANGLE);
            if(m_controller.getCrossButtonReleased())
                setWantedState(SystemState.NEUTRAL);

            if(m_controller.getCircleButtonPressed())
                setWantedState(SystemState.MID);

            if(m_controller.getTriangleButtonPressed())
                setWantedState(SystemState.NEUTRAL);

            if(m_controller.getSquareButtonPressed())
                setWantedState(SystemState.HIGH);

            if(m_controller.getR1ButtonPressed())
                setWantedState(SystemState.AUTON_HIGH); // hUMAN fOLD
            if(m_controller.getR1ButtonReleased())
                setWantedState(SystemState.NEUTRAL);
            if (m_controller.getOptionsButtonPressed())
                setWantedState(SystemState.ZERO);
            
        } else {
            if (m_controller.getOptionsButtonPressed()){
                // TODO - what is this doing?
                // m_rotateMotor.setSelectedSensorPosition(0);
            }
        }

        if (m_controller.getPSButtonPressed()){
            
            if (m_currentState == SystemState.MANUAL){
                setWantedState(SystemState.NEUTRAL);
                m_manualMode = false;
            }else{
                setWantedState(SystemState.MANUAL);
                m_manualMode= true;
            }
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        switch (m_currentState){
            case GROUND_ANGLE:

				//4096 ticks in a revolution
				// configRotate(-20.8);   //-85190/4096
                configRotateAngle(-90);
				configExtend(0);	
                break;
             case MID:			
				// configRotate(-10.2);  //-42080/4096
                configRotateAngle(-45);
                configExtend(9.7);    //39949/4096
                break;
            case HIGH:
				// configRotate(-9.6);   //-39320/4096
                configRotateAngle(-45);
				configExtend(28.3);     //116256/4096
                break;
            case AUTON_MID:
				// TODO - check this rotate.  shouldn't just be opposite MID ?
                // configRotate(11.2);   //46080/4096
                configRotateAngle(45);
                configExtend(9.7);    //39949/4096
                break;
            case AUTON_HIGH:
				//TODO - check this rotate.  shouldn't just be opposite HIGH ?
				// configRotate(9.1);    //(41320-4000)/4096
                configRotateAngle(45);
				configExtend(27.4);     //112256/4096
                break;
            // case MANUAL:
            //     manualControl(m_controller.getLeftX(), -m_controller.getRightY());
            //     break;
            case HUMAN_FOLD:
				// configRotate(-10.1);   //-41320/4096
                configRotateAngle(-40);
                configExtend(0);
                break;
            case ZERO:
				// configRotate(17.1);   //70057/4096
                configRotateAngle(22);
                configExtend(0);
                break;
            default:
            case NEUTRAL:
                // neutralize();
                // configRotate(0);
                configRotateAngle(0);
                configExtend(0);
                break;
            
        }
    }

    // public void neutralize(){
    //     if (m_rotateLimitSwitch.get()){
    //         if (m_rotateMotor.getSelectedSensorPosition() < 0){
    //             m_rotateMotor.set(ControlMode.PercentOutput, 0.4);
    //         } else {
    //             m_rotateMotor.set(ControlMode.PercentOutput, -0.4); 
    //         }
    //     } else {
    //         m_rotateMotor.set(ControlMode.PercentOutput, 0);
    //     }
    // }

    @Override
    public void stop() {
        
    }

    @Override
    public void outputTelemetry(double timestamp){
        SmartDashboard.putBoolean("ExtendLimitSwitch", m_extendLimitSwitch.get());
        SmartDashboard.putBoolean("RotateLimitSwitch", m_rotateLimitSwitch.get());
        SmartDashboard.putString("Extend Motor Pos", m_extendMotor.getPosition().toString());
        SmartDashboard.putString("Rotate Motor Pos", m_rotateMotor.getPosition().toString());
		SmartDashboard.putString("Extend Motor Temp", m_extendMotor.getDeviceTemp().toString());
		SmartDashboard.putString("Rotate Motor Temp", m_rotateMotor.getDeviceTemp().toString());
        SmartDashboard.putBoolean("Manual Mode", m_manualMode);
    }

    private SystemState handleManual(){
        return m_wantedState;
    }

    public void setWantedState(SystemState wanted){
        m_wantedState = wanted;
    }

    public void configExtend(double position){
        m_extendMotor.setControl(m_extendMotorMMV.withPosition(position));
    }
   
    public void configRotate(double position){
        m_rotateMotor.setControl(m_rotateMotorMMV.withPosition(position));
    }

    public void configRotateAngle(double angle){
        //4096 ticks
        //0 position is ARM straight-up
        double position = Constants.ARM.ROTATEENCODERZEROOFFSET + angle * 4096;
        configRotate(position);
    }
    
    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
	    zeroExtendSensor();
        zeroRotateSensor();
    }
   
    public void zeroExtendSensor(){
        m_extendMotor.setControl(m_extendMotorMMV.withPosition(0));
    }
    public void zeroRotateSensor(){
        //move arm to vertical '0' position
        m_rotateMotor.setControl(m_rotateMotorMMV.withPosition(Constants.ARM.ROTATEENCODERZEROOFFSET));
    }


    // public void manualControl(double rotateOutput, double armOutput){
    //     m_rotateMotor.set(ControlMode.PercentOutput, rotateOutput/4);
    //     if (!m_extendLimitSwitch.get() && armOutput < 0){
    //         m_extendMotor.set(ControlMode.PercentOutput, 0);
    //     } else {
    //         m_extendMotor.set(ControlMode.PercentOutput, armOutput);
    //     }
        
    // }


    @Override
    public String getId() {
        return "Arm";
    }

    
}