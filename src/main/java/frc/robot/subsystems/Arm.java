package frc.robot.subsystems;


import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
// import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Arm implements Subsystem {

/*  MANIPULATOR CONTROLLER SCHEME (PLEASE KEEP UPDATED)
 * 
 *  MAPPED BUTTONS:
 *  Triangle -> Neutral Position
 *  Cross -> Low Position
 *  Circle -> Mid Position
 *  Square -> High Position
 *  RBumper -> Human Fold Angle
 *  Options -> Zero Motor Encoder by Limit Switch
 *  
 * 
 *  PS4 Button -> Manual Mode Toggle (Info on SmartDashboard)
 *  WHEN IN MANUAL:
 *      RYAxis -> Extend Control
 *      LXAxis -> Rotate Control
 *      Options -> Zero the Motor Encoder
 *  
 */


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
        TRANSITION,
        SHOOT,
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
    protected VoltageOut m_rotateVoltageOut;

    protected double m_rotate_angle;
    protected double m_rotate_rotations;
    protected double m_extend_rotations;
    protected boolean m_extendLimitSwitchHit = false;
    // protected double m_extendLimitSwitch_debonce = 0;
    protected boolean m_rotateLimitSwitchHit = false;
    // protected double m_rotateLimitSwitch_debonce = 0;
    // protected boolean m_rotateSwitchStuck = false;

    protected double m_rotateEncoderAngle = 0;
    protected double m_rotateEncoderVelocity = 0;
    protected double m_rotateEncoderRotations = 0;


    private final Intake m_intake;
    private final PS4Controller m_controller;
    private final PowerDistribution m_PDH; 

    private boolean m_manualMode = false;

    public Arm(PS4Controller controller, Intake intake){

        m_intake = intake;
        m_controller = controller;

        m_PDH = new PowerDistribution();

        // extendEncoderInit();
		extendMotorInit();
        m_extendLimitSwitch = new DigitalInput(9);

        //---------------------------------------------------------------------
        rotateEncoderInit();
        rotateMotorInit();
		m_rotateLimitSwitch = new DigitalInput(0);

    }

    private void extendMotorInit(){
        m_extendMotor = new TalonFX(Constants.ARM.EXTENDMOTOR, "MANIPbus");
        m_extendMotorMMV = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 80; //106; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 100; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 700;   
	
        cfg.Slot0.kP = 55.0F;
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

        // cfg.Voltage.PeakForwardVoltage = 4.0;
        // cfg.Voltage.PeakForwardVoltage = -4.0;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 30.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_extendMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure extend motor. Error: " + status.toString());
        }

        m_extendMotorMMV.OverrideBrakeDurNeutral = true;
        m_extendMotor.setVoltage(0);
        // m_extendMotor.setSafetyEnabled(false);

        zeroExtendSensor();
    }

	private void rotateMotorInit(){
        m_rotateMotor = new TalonFX(Constants.ARM.ROTATEMOTOR, "MANIPbus");
        m_rotateMotorMMV = new MotionMagicVoltage(0);  

        // for manual control we need to use voltageout
        m_rotateVoltageOut = new VoltageOut(0).withOverrideBrakeDurNeutral(true);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 80; //106; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 100; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 700;   

		m_rotateMotor.setInverted(true);
        
        cfg.Slot0.kP = 55.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving
    
        
        // // tie CANcode on arm rotate shaft to the left motor
        // cfg.Feedback.FeedbackRemoteSensorID = m_rotateEncoder.getDeviceID();
        // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // // cfg.Feedback.SensorToMechanismRatio = 2F;
        
        // cfg.Voltage.PeakForwardVoltage = 4.0;
        // cfg.Voltage.PeakForwardVoltage = -4.0;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 30.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_rotateMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure rotate motor. Error: " + status.toString());
        }

        m_rotateMotorMMV.OverrideBrakeDurNeutral = true;
        m_rotateMotor.setVoltage(0);
        // m_rotateMotor.setSafetyEnabled(false);

        zeroRotateSensor(); 
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
    
        // cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // cc_cfg.MagnetSensor.MagnetOffset = 0.1; //0; // 3.24; //0; //-2.82; //-1.82; //-1.2; //-1.52; //-1.77; // -1.866; //-1.74; //-1.82;
        m_rotateEncoder.getConfigurator().apply(cc_cfg);

        /* Speed up signals to an appropriate rate */
        // m_rotateEncoder.getPosition().setUpdateFrequency(100);
        // m_rotateEncoder.getVelocity().setUpdateFrequency(100);

        zeroRotateEncoder();
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
            case TRANSITION:
                newState = handleManual();
                break;
            case AUTON_MID:
                newState = handleManual();
                break;
            case AUTON_HIGH:
                newState = handleManual();
                break;
            case SHOOT:
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
  

        m_extendLimitSwitchHit = m_extendLimitSwitch.get();
        m_rotateLimitSwitchHit = m_rotateLimitSwitch.get();

        m_rotateEncoderAngle = m_rotateEncoder.getAbsolutePosition().getValue();
        m_rotateEncoderVelocity = m_rotateEncoder.getVelocity().getValue();

        m_rotateEncoderRotations = m_rotateEncoderAngle*142.22;

       if(!m_extendLimitSwitchHit){
           zeroExtendSensor();
       }

    //    if(!m_rotateLimitSwitchHit){
    //         //limit switch hit, position rotor should be at this point
    //         //TODO - should this be -1.26?  or this is hit going backwards, so stay positive.
    //         //  also should we be doing this every loop? and every time passed?
    //         //  does this mess-up motion magic?
    //         //  maybe should be only in manual mode?
            
    //         if (m_rotateLimitSwitch_debonce < 5 ){
    //             m_rotateLimitSwitch_debonce++;

    //         } else if (m_rotateLimitSwitch_debonce == 5 ){
    //             //we have a pulse low for 5 counts ( 5 * 20ms = 100ms)    
    //             m_rotateMotor.setRotorPosition(2.6);
    //             m_rotateLimitSwitch_debonce = 0;
    //         } else {
    //             // must be stuck,  do nothing
    //             m_rotateSwitchStuck = true;
    //         }
    //     } else {
    //         //switch is open, so count back to zero and stay there while open
    //         if (m_rotateLimitSwitch_debonce > 0)
    //             m_rotateLimitSwitch_debonce--;
    //     }
    

        if (!(m_currentState == SystemState.MANUAL)){
            // if(m_intake.getIntakeCurrent()>=200 && m_intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.PLACING && m_intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.IDLE)
               // setWantedState(SystemState.NEUTRAL);

            if(m_controller.getCrossButtonPressed())
                setWantedState(SystemState.GROUND_ANGLE);
            if(m_controller.getCrossButtonReleased())
                setWantedState(SystemState.NEUTRAL);

            if(m_controller.getCircleButtonPressed())
                setWantedState(SystemState.MID);

            if(m_controller.getTriangleButtonPressed()){
                if (m_currentState == SystemState.HIGH)
                    new SequentialCommandGroup(new InstantCommand(() -> setWantedState(SystemState.TRANSITION)),
                        new WaitCommand(0.5),
                        new InstantCommand(() -> setWantedState(SystemState.NEUTRAL))).schedule();
                else
                    setWantedState(SystemState.NEUTRAL);
            }

            if(m_controller.getSquareButtonPressed())
                setWantedState(SystemState.HIGH);

            if(m_controller.getR1ButtonPressed())
                setWantedState(SystemState.HUMAN_FOLD); // hUMAN fOLD
            if(m_controller.getR1ButtonReleased())
                setWantedState(SystemState.NEUTRAL);

            if (m_controller.getOptionsButtonPressed())
                syncRotateMotor();
        }

		else if (m_controller.getOptionsButtonPressed())
        	zeroRotateEncoder();
			
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
				configRotate(-43.5);  //-20.8);   //-85190/4096
                // configRotateAngle(-110);
				configExtend(0);	
                break;
             case MID:			
				configRotate(-24.0);  // -10.2);  //-42080/4096
                // configRotateAngle(-45);   //TODO: tweak angle
                configExtend(24.0);    //39949/4096
                break;
            case HIGH:
				configRotate(-21.0); // -9.6);   //-39320/4096
                // configRotateAngle(45);   //TODO: tweak angle
				configExtend(57.0);  //61.5);     //116256/4096
                break;
            case AUTON_MID:
	            configRotate(19.0); // 11.2);   //46080/4096
                // configRotateAngle(-45);   //TODO: tweak angle
                configExtend(24.0);    //39949/4096
                break;
            case AUTON_HIGH:
				configRotate(19.0);    //(41320-4000)/4096
                // configRotateAngle(-45);   //TODO: tweak angle
				configExtend(57.0);  //62.5);  //27.41);     //112256/4096
                break;
            case TRANSITION:
                configExtend(24.0);
                configRotate(-21.0);
                break;
            case MANUAL:
                manualControl(m_controller.getLeftX(), -m_controller.getRightY());
                break;
            case HUMAN_FOLD:
				configRotate(-24.0); // -10.1);   //-41320/4096
                // configRotateAngle(40);   //TODO: tweak angle
                configExtend(0);
                break;
            case SHOOT:
                configExtend(-19.0);
                configRotate(24.0);
            // case ZERO:
			// 	//TODO how does this work? 
            //     configRotate(24.0);   //70057/4096
            //     // configRotateAngle(-45.10);
            //     configExtend(0);
            //     break;
            default:
            case NEUTRAL:
                // neutralize();
                configRotate(0);
                // configRotateAngle(0);
                configExtend(0);
                break;
            
        }
    }

    private void syncRotateMotor(){
        // only do this when in a known location (i.e. not while arm moving)
        // And only do this if not too far off (because too far off suggests bad encoder)
        // if (Math.abs(m_rotateEncoderRotations-m_rotate_rotations) <2){
            m_rotateMotor.setRotorPosition(m_rotateEncoderRotations);
        // }
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
        SmartDashboard.putString("Arm State", m_currentState.toString());
        SmartDashboard.putBoolean("ExtendLimitSwitch", m_extendLimitSwitchHit);
        SmartDashboard.putBoolean("RotateLimitSwitch", m_rotateLimitSwitchHit);
        SmartDashboard.putNumber("Extend Motor Pos", m_extendMotor.getPosition().getValue());
        SmartDashboard.putNumber("Rotate Motor Pos", m_rotateMotor.getPosition().getValue());
		SmartDashboard.putNumber("Extend Motor Temp", m_extendMotor.getDeviceTemp().getValue());
		SmartDashboard.putNumber("Rotate Motor Temp", m_rotateMotor.getDeviceTemp().getValue());
        // SmartDashboard.putNumber("rotate angle commanded", m_rotate_angle);
        SmartDashboard.putNumber("Rotate setpoint", m_rotate_rotations);
        SmartDashboard.putNumber("Extend setpoint", m_extend_rotations);
        SmartDashboard.putBoolean("Manual Mode", m_manualMode);
        // SmartDashboard.putString("rotate encoder pos", m_rotateEncoder.getPosition().toString());
        // SmartDashboard.putBoolean("Ext Motor sfty en", m_extendMotor.isSafetyEnabled());
        // SmartDashboard.putBoolean("Rot Motor sfty en", m_rotateMotor.isSafetyEnabled());
        SmartDashboard.putNumber("Ext Motor sup cur", m_extendMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("Rot Motor sup cur", m_rotateMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("Arm Ecdr Angle", m_rotateEncoderAngle);
        SmartDashboard.putNumber("Arm Ecdr Rot", m_rotateEncoderRotations);
        SmartDashboard.putNumber("Arm Ecdr Velocity", m_rotateEncoderVelocity);
        SmartDashboard.putNumber("PDH Voltage", m_PDH.getVoltage());
        SmartDashboard.putNumber("PDH Current", m_PDH.getTotalCurrent());
    }

    private SystemState handleManual(){
        return m_wantedState;
    }

    public void setWantedState(SystemState wanted){
        m_wantedState = wanted;
    }

    public void configExtend(double position){
        //position is number of rotations, not number of ticks
        m_extend_rotations = position;
        m_extendMotor.setControl(m_extendMotorMMV.withPosition(position));
    }
   
    public void configRotate(double position){
        //position is number of rotations, not number of ticks
        m_rotate_rotations = position;
        m_rotateMotor.setControl(m_rotateMotorMMV.withPosition(position));
    }

    // public void configRotateAngle(double angle){
    //     m_rotate_angle = angle;
    //     m_rotate_rotations = (m_rotate_angle) / 360;
    //     configRotate(m_rotate_rotations);
    // }
    
    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
	    zeroExtendSensor();
        zeroRotateSensor();
        zeroRotateEncoder();
    }
   
    public void zeroExtendSensor(){
        m_extendMotor.setRotorPosition(0);
    }

    public void zeroRotateSensor(){
        m_rotateMotor.setRotorPosition(0);
        // m_rotateEncoder.setPosition(0);
        //move arm to vertical '0' position
        // configRotateAngle(0);
        //  No need to zero.   absolute CAN coder position will be used.
        //  so if it starts off zero, it will go to zero upon going to Nuetral state 
        //NOPE, magnet offset did not work.   go back to set rotor to zero.
    }

    public void zeroRotateEncoder(){
        m_rotateEncoder.setPosition(0);
    }


    public void manualControl(double rotatePercentOutput, double armPercentOutput){
        m_rotateMotor.setControl(m_rotateVoltageOut.withOutput(Constants.ARM.MAX_MANUAL_SUPPLY_VOLTAGE*rotatePercentOutput/4));

        if (!m_extendLimitSwitch.get() && armPercentOutput < 0){
            m_extendMotor.setControl(m_rotateVoltageOut.withOutput(0));
        } else {
            m_extendMotor.setControl(m_rotateVoltageOut.withOutput(Constants.ARM.MAX_MANUAL_SUPPLY_VOLTAGE*armPercentOutput));
        }
    }


    @Override
    public String getId() {
        return "Arm";
    }

    
}