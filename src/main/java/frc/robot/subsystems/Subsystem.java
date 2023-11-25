package frc.robot.subsystems;

public interface Subsystem extends edu.wpi.first.wpilibj2.command.Subsystem {

	void processLoop(double timestamp);

	void stop();

	boolean checkSystem();

	void zeroSensors();

	String getId();

	default void readPeriodicInputs(double timestamp) {

	}

	default void writePeriodicOutputs(double timestamp) {

	}

	default void outputTelemetry(double timestamp){

	}

    default void longLoop(double timestamp) {
        
    }
}

