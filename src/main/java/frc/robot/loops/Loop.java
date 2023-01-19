package frc.robot.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic
 * gyroscope calibration, etc.)
 * 
 * FROM TEAM 27 RUSH
 */
public interface Loop {

	void onStart(double timestamp);

	void onLoop(double timestamp);

	void onStop(double timestamp);

	String getId();
}

