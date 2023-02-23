package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake implements Subsystem {


    private enum SystemState{
        IDLE,   
        INTAKING,
        PLACING
        
    }

    public enum WantedState{
        IDLE,
        INTAKING,
        PLACING
    }

    private SystemState currentState = SystemState.IDLE;
    private WantedState wantedState = WantedState.IDLE;

    private double currentStateStartTime = 0;

    private TalonFX intakeMotor;

    private final XboxController controller;


    public Intake(XboxController controller){
        intakeMotor = new TalonFX(Constants.INTAKE.INTAKE_MOTOR);

        intakeMotor.configPeakOutputForward(1);
        intakeMotor.configPeakOutputReverse(-1);
        intakeMotor.setNeutralMode(NeutralMode.Brake); 
        this.controller = controller;
    }

    @Override
    public void processLoop(double timestamp) {
        // TODO Auto-generated method stub
        SystemState newState;
        switch (currentState){
            default:
            case INTAKING:
                newState = handleManual();
                break;
            case IDLE:
                newState = handleManual();
                break;

        }
        if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
		}
    }

    @Override
    public void readPeriodicInputs(double timestamp){
        if (controller.getAButtonPressed())
            wantedState = (currentState != SystemState.IDLE) ? WantedState.IDLE : WantedState.INTAKING;

        if (currentState == SystemState.INTAKING && getIntakeCurrent() > 150)
            wantedState = WantedState.IDLE;

        if (controller.getBButtonPressed())
            wantedState = (currentState != SystemState.IDLE) ? WantedState.IDLE : WantedState.PLACING;


    }

    @Override
    public void writePeriodicOutputs(double timestamp){
        switch(currentState){

            case INTAKING:
                setIntakeSpeed(-.5);
                break;
            case PLACING:
                setIntakeSpeed(.5);
                break;
            default:
            case IDLE:
                setIntakeSpeed(0);
                break;
        }
    }


    private SystemState handleManual(){
        switch (wantedState){
            case INTAKING:
                return SystemState.INTAKING;
            case PLACING:
                return SystemState.PLACING;
            default:
            case IDLE:
                return SystemState.IDLE;
		}
    }

    public void setIntakeSpeed(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }
    public double getIntakeCurrent(){
        return intakeMotor.getStatorCurrent();
    }
    @Override
    public void outputTelemetry(double timestamp){
        SmartDashboard.putNumber("Current Stator Current", getIntakeCurrent());
        SmartDashboard.putNumber("Supply Current", intakeMotor.getSupplyCurrent());
    }
    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub

    }

    @Override
    public String getId() {
        // TODO Auto-generated method stub
        return "Intake";
    }

    public void setWantedState(WantedState wantedState) {
		this.wantedState = wantedState;
	}

}