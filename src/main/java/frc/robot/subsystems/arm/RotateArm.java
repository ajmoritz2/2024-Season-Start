package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class RotateArm implements Subsystem {

    private TalonFX rotateMotorLeft;
    private TalonFX rotateMotorRight;
    private CANCoder magEncoder;

    private enum SystemState{
        NEUTRAL,
        HOLD,
        ROTATER,
        ROTATEL;
    }

    public enum WantedState{
        NEUTRAL,
        HOLD,
        ROTATER,
        ROTATEL;
    }

    private SystemState currentState = SystemState.NEUTRAL;
    private WantedState wantedState = WantedState.NEUTRAL;

    private double currentStateStartTime = 0;

    private final XboxController controller;

    public RotateArm(XboxController controller){

        this.controller = controller;

        rotateMotorLeft = new TalonFX(Constants.Arm.ROTATEMOTOR1);
        rotateMotorRight = new TalonFX(Constants.Arm.ROTATEMOTOR2);
        rotateMotorRight.follow(rotateMotorLeft, FollowerType.PercentOutput);
        magEncoder = new CANCoder(Constants.Arm.EXTENDENCODER);
        
    }

    @Override
    public void processLoop(double timestamp) {
        SystemState newState;
        switch(currentState){
            default:
            case NEUTRAL:
                newState = handleManual();
                break;
            case ROTATER:
                newState = handleManual();
                break;
            case ROTATEL:
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
       if (controller.getYButtonReleased())
            wantedState = (currentState != SystemState.HOLD) ? WantedState.HOLD : WantedState.ROTATER; 
       

       if (controller.getXButtonReleased())
        wantedState = (currentState != SystemState.HOLD) ? WantedState.HOLD : WantedState.ROTATEL;  
    }

    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        switch (currentState){
            default:
            case NEUTRAL:
                configExtend(0);
                break;
            case ROTATEL:
                configExtend(1);
                break;
            case ROTATER:
                configExtend(-1);
                break;
            case HOLD:
                configExtend(0);
                rotateMotorLeft.setNeutralMode(NeutralMode.Brake);
                rotateMotorRight.setNeutralMode(NeutralMode.Brake);
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
            case ROTATEL:
                return SystemState.ROTATEL;
            case ROTATER:
                return SystemState.ROTATER;
            default:
                return SystemState.NEUTRAL;

        }
    }

    public void extendTick(int position){
        int dir = (position > magEncoder.getPosition()) ? 1 : -1; // Left -> 1 | Right -> -1 Subject to change

        if ((magEncoder.getPosition() <= position + Constants.Arm.ROTATE_DEADRANGE &&
            magEncoder.getPosition() >= position - Constants.Arm.ROTATE_DEADRANGE)){
                wantedState = WantedState.HOLD;
                return;
        }
        
        configExtend(dir);
    }

    public void configExtend(float speed){
        rotateMotorLeft.setNeutralMode(NeutralMode.Coast);
        rotateMotorLeft.set(ControlMode.PercentOutput,ensureRange(speed,-Constants.Arm.MAX_ROTATO_SPEED,Constants.Arm.MAX_ROTATO_SPEED));
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
