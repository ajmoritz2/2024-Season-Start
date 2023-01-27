package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Arm implements Subsystem {

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

    public Arm(XboxController controller){

        this.controller = controller;

        liftMotor = new TalonFX(Constants.Arm.EXTENDMOTOR);
        magEncoder = new CANCoder(Constants.Arm.EXTENDENCODER);

        liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10,15,0.5));
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
            wantedState = (currentState != SystemState.NEUTRAL) ? WantedState.NEUTRAL : WantedState.EXTEND; 
       

       if (controller.getBButtonReleased())
        wantedState = (currentState != SystemState.NEUTRAL) ? WantedState.NEUTRAL : WantedState.RETRACT;  
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
                liftMotor.setNeutralMode(NeutralMode.Brake);
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

    public void configExtend(float speed){
        liftMotor.setNeutralMode(NeutralMode.Coast);
        liftMotor.set(ControlMode.PercentOutput,ensureRange(speed,-Constants.Arm.MAXEXTENDOSPEED,Constants.Arm.MAXEXTENDOSPEED));
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
        magEncoder.setPosition(0);
    }

    @Override
    public String getId() {
        return null;
    }

    private float ensureRange(float value, float min, float max) {
        return Math.min(Math.max(value, min), max);
     }
    
}
