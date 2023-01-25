package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class Arm implements Subsystem{

    private TalonFX liftMotor;
    private TalonFX rotateMotorLeft;
    private TalonFX rotateMotorRight;
    private CANCoder magEncoder;

    private enum SystemState{

    }

    public enum WantedState{

    }

    private final XboxController controller;

    public Arm(XboxController controller){

        this.controller = controller;
    }

    @Override
    public void processLoop(double timestamp) {
        // TODO Auto-generated method stub
        
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
        return null;
    }
    
}
