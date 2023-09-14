package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.littletonrobotics.junction.Logger;

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
        PLACING,
        SHOOT
    }

    public enum WantedState{
        IDLE,
        INTAKING_CONE,
        INTAKING_CUBE,
        IDLE_CUBE,
        PLACING,
        SHOOT
    }

    private SystemState currentState = SystemState.IDLE;
    private WantedState wantedState = WantedState.IDLE;
    

    private boolean haveCube;
    private boolean haveCone;

    private double currentStateStartTime = 0;

    private TalonFX intakeMotor;

    private final PS4Controller controller;

    private double m_IntakeStatorCurrent = 0;
    private double m_intakePercentOut = 0;


    public Intake(PS4Controller controller){
        intakeMotor = new TalonFX(Constants.INTAKE.INTAKE_MOTOR, "MANIPbus");

        
        intakeMotor.configPeakOutputForward(1);
        intakeMotor.configPeakOutputReverse(-1);
        intakeMotor.setInverted(true);
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
            case SHOOT:
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

        m_IntakeStatorCurrent = intakeMotor.getStatorCurrent();

        if (controller.getL1ButtonPressed())
            wantedState =  WantedState.INTAKING_CONE;
        if (controller.getL1ButtonReleased())
            wantedState = WantedState.IDLE;

        if (controller.getL2ButtonPressed())
            wantedState = WantedState.INTAKING_CUBE;
        if (controller.getL2ButtonReleased())
            wantedState = WantedState.IDLE;

        if (currentState == SystemState.INTAKING_CONE && m_IntakeStatorCurrent > 200){
            // new SequentialCommandGroup(
            //     new WaitCommand(5),
            //     new InstantCommand(()-> setWantedState(WantedState.IDLE))
            // );
            haveCone = true;
        }
        if (currentState == SystemState.INTAKING_CUBE && m_IntakeStatorCurrent > 100){
            // new SequentialCommandGroup(
            //     new WaitCommand(5),
            //     new InstantCommand(()-> setWantedState(WantedState.IDLE_CUBE))
            // );
            haveCube = true;
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
                setIntakeSpeed(-1);
                break;
            case INTAKING_CUBE:
                setIntakeSpeed(-.3);
                break;
            case PLACING:
                setIntakeSpeed(.15);
                break;
            case IDLE_CUBE:
                setIntakeSpeed(-.1);
                break;
            case SHOOT:
                setIntakeSpeed(1);
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
            case SHOOT:
                return SystemState.SHOOT;
            default:
            case IDLE:
                return SystemState.IDLE;
		}
    }

    public void setIntakeSpeed(double speed){
        m_intakePercentOut = speed;
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }
    public double getIntakeCurrent(){
        return m_IntakeStatorCurrent;
    }
    @Override
    public void outputTelemetry(double timestamp){
        Logger log = Logger.getInstance();

        log.recordOutput("Intake State", currentState.toString());
        log.recordOutput("Intake Stator Cur", m_IntakeStatorCurrent);
        log.recordOutput("Intake Supply Cur", intakeMotor.getSupplyCurrent());
        log.recordOutput("Intake Motor Temp", intakeMotor.getTemperature());
        log.recordOutput("Intake Percent Out", m_intakePercentOut);
        log.recordOutput("HaveCube", haveCube);
        log.recordOutput("HaveCone", haveCone);

        SmartDashboard.putString("Intake State", currentState.toString());
        SmartDashboard.putNumber("Intake Stator Cur", m_IntakeStatorCurrent);
        SmartDashboard.putNumber("Intake Supply Cur", intakeMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Intake Motor Temp", intakeMotor.getTemperature());
        SmartDashboard.putNumber("Intake Percent Out", m_intakePercentOut);
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