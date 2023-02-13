package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class ExtendArm implements Subsystem {

    private TalonFX liftMotor;
    private TalonFX rotateMotorLeft;
    private TalonFX rotateMotorRight;
    private CANCoder magEncoder;

    private enum SystemState{
        NEUTRAL,
        HOLD,
        EXTEND,
        RETRACT;
    }

    public enum WantedState{
        NEUTRAL,
        HOLD,
        EXTEND,
        RETRACT;
    }

    private SystemState currentState = SystemState.NEUTRAL;
    private WantedState wantedState = WantedState.NEUTRAL;

    private double currentStateStartTime = 0;

    private final XboxController controller;

    public ExtendArm(XboxController controller){

        this.controller = controller;

        liftMotor = new TalonFX(Constants.Arm.EXTENDMOTOR);

        var talonFXConfigs = new TalonFXConfiguration();

        // I really dont wanna go any farther with these until we can test because this is very out there. I dont know what is going on at all here and sdfsojonnsgarngrdfdag
        
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 24; // A position error of 0.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output 
        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 8; // Target cruise velocity of 8 rps
        motionMagicConfigs.MotionMagicAcceleration = 40; // Target acceleration of 40 rps/s
        motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 800 rps/s/s

        liftMotor.getConfigurator().apply(talonFXConfigs); 
        
    }

    @Override
    public void processLoop(double timestamp) {
        SystemState newState;
        switch(currentState){
            default:
            case NEUTRAL:
                newState = handleManual();
                break;
            case EXTEND:
                newState = handleManual();
                break;
            case RETRACT:
                newState = handleManual();
                break;
            case HOLD:
                newState = handleManual();
                break;
        }

        if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
		}
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        // Yes I am going to use short hand if statements because it looks better.
        // (condition) ? true : false;

        // I also do != Neutral because that should make sure it isn't in some weird mode and have some motors clash
       if (controller.getAButtonReleased())
            wantedState = (currentState != SystemState.HOLD) ? WantedState.HOLD : WantedState.EXTEND; 
       

       if (controller.getBButtonReleased()){
            wantedState = (currentState != SystemState.HOLD) ? WantedState.HOLD : WantedState.RETRACT;  
            System.out.println("BBBBBB");
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        switch (currentState){
            default:
            case NEUTRAL:
                configExtend(0);
                break;
            case EXTEND:
                configExtend(1);
                break;
            case RETRACT:
                configExtend(-1);
                break;
            case HOLD:
                configExtend(0);
                
                break;
        }
    }

    @Override
    public void stop() {
        configExtend(0);
        zeroSensors();
    }

    private SystemState handleManual(){
        switch (wantedState){
            case HOLD:
                return SystemState.HOLD;
            case NEUTRAL:
                return SystemState.NEUTRAL;
            case EXTEND:
                return SystemState.EXTEND;
            case RETRACT:
                return SystemState.RETRACT;
            default:
                return SystemState.NEUTRAL;

        }
    }

    public void extendTick(int position){
        var request = new MotionMagicVoltage(0).withSlot(0);

        // set position to 10 rotations
        liftMotor.setControl(request.withPosition(position));
    }

    public void configExtend(float speed){
        var request = new VelocityVoltage(0).withSlot(0);
        liftMotor.setControl(request.withVelocity(speed).withFeedForward(0.5));
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public String getId() {
        return null;
    }

    private float ensureRange(float value, float min, float max) {
        return Math.min(Math.max(value, min), max);
     }
    
}
