package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
	public static final RobotMode CURRENT_MODE = RobotBase.isSimulation() ? RobotMode.SIM : RobotMode.REAL;

    public enum RobotMode {
		REAL, // Running on a real robot
		SIM, // Running a physics simulator
		REPLAY // Replaying from a log file
	}
}
