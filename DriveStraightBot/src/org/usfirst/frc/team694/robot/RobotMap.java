package org.usfirst.frc.team694.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface RobotMap {

	//TODO: Set. Look at code from the robot you're using
	double DRIVETRAIN_ENCODERS_INCHES_PER_REVOLUTION = -1;
	double DRIVETRAIN_ENCODERS_FACTOR = -1;

	// Drivetrain Motor Ports
	int LEFT_FRONT_MOTOR_PORT = 1;
	int RIGHT_FRONT_MOTOR_PORT = 1;
	int LEFT_BACK_MOTOR_PORT = 1;
	int RIGHT_BACK_MOTOR_PORT = 1;

	// Gamepad Ports
	int GAMEPAD_DRIVER_PORT = 0;

}
