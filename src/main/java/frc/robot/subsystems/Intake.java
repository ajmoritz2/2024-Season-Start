package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Intake implements Subsystem {


    public enum SystemState{
        IDLE,
        INTAKING_CONE,
        INTAKING_CUBE,
        IDLE_CUBE,
        PLACING
    }

    public enum WantedState{
        IDLE,
        INTAKING_CONE,
        INTAKING_CUBE,
        IDLE_CUBE,
        PLACING
    }

    private SystemState currentState = SystemState.IDLE;
    private WantedState wantedState = WantedState.IDLE;

    private boolean haveCube;
    private boolean haveCone;

    private double currentStateStartTime = 0;

    private TalonFX intakeMotor;

    private final PS4Controller controller;


    public Intake(PS4Controller controller){
        intakeMotor = new TalonFX(Constants.INTAKE.INTAKE_MOTOR, "MANIPbus");

        
        intakeMotor.configPeakOutputForward(1);
        intakeMotor.configPeakOutputReverse(-1);
        intakeMotor.setNeutralMode(NeutralMode.Brake); 
        haveCone = false;
        haveCube = false;

        this.controller = controller;
    }

    @Override
    public void processLoop(double timestamp) {
        // TODO Auto-generated method stub
        SystemState newState;
        switch (currentState){
            default:
            case INTAKING_CONE:
                newState = handleManual();
                break;
            case INTAKING_CUBE:
                newState = handleManual();
                break;
            case PLACING:
                newState = handleManual();
                break;
            case IDLE_CUBE:
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
        if (controller.getL1ButtonPressed())
            wantedState =  WantedState.INTAKING_CONE;
        if (controller.getL1ButtonReleased())
            wantedState = WantedState.IDLE;

        if (controller.getL2ButtonPressed())
            wantedState = WantedState.INTAKING_CUBE;
        if (controller.getL2ButtonReleased())
            wantedState = WantedState.IDLE;

        if (currentState == SystemState.INTAKING_CONE && getIntakeCurrent() > 200){
            new SequentialCommandGroup(
                new WaitCommand(5),
                new InstantCommand(()-> setWantedState(WantedState.IDLE))
            );
            haveCone = true;
        }
        if (currentState == SystemState.INTAKING_CUBE && getIntakeCurrent() > 100){
            new SequentialCommandGroup(
                new WaitCommand(5),
                new InstantCommand(()-> setWantedState(WantedState.IDLE_CUBE))
            );
            haveCube = true;
        }

        if(haveCone){
            wantedState = WantedState.IDLE;
            if (controller.getL1ButtonPressed())
                wantedState =  WantedState.INTAKING_CONE;
        }
        if (haveCube){
            wantedState = WantedState.IDLE_CUBE;
            if (controller.getL2ButtonPressed())
                wantedState = WantedState.INTAKING_CUBE;
        }

            
        if (controller.getR2ButtonPressed()){
            wantedState = WantedState.PLACING;
            haveCone = false;
            haveCube = false;
        }
            
        if (controller.getR2ButtonReleased())
            wantedState = WantedState.IDLE;


    }

    @Override
    public void writePeriodicOutputs(double timestamp){
        switch(currentState){

            case INTAKING_CONE:
                setIntakeSpeed(-.7);
                break;
            case INTAKING_CUBE:
                setIntakeSpeed(-.3);
                break;
            case PLACING:
                setIntakeSpeed(.5);
                break;
            case IDLE_CUBE:
                setIntakeSpeed(-.1);
                break;
            default:
            case IDLE:
                setIntakeSpeed(0);
                break;
        }
    }


    private SystemState handleManual(){
        switch (wantedState){
            case INTAKING_CONE:
                return SystemState.INTAKING_CONE;
            case INTAKING_CUBE:
                return SystemState.INTAKING_CUBE;
            case PLACING:
                return SystemState.PLACING;
            case IDLE_CUBE:
                return SystemState.IDLE_CUBE;
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
        SmartDashboard.putBoolean("HaveCube", haveCube);
        SmartDashboard.putBoolean("HaveCone", haveCone);
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

    public SystemState getCurrentState(){
        return currentState;
    }

}