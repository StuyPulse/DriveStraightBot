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
	int LEFT_FRONT_MOTOR_PORT = 4;
	int RIGHT_FRONT_MOTOR_PORT = 1;
	int LEFT_BACK_MOTOR_PORT = 3;
	int RIGHT_BACK_MOTOR_PORT = 2;

	// Drivetrain Encoder Ports
    int DRIVETRAIN_ENCODER_LEFT_CHANNEL_A = 2;
    int DRIVETRAIN_ENCODER_LEFT_CHANNEL_B = 3;
    int DRIVETRAIN_ENCODER_RIGHT_CHANNEL_A = 0;
    int DRIVETRAIN_ENCODER_RIGHT_CHANNEL_B = 1;

    // Other Motor Ports
    int SHOOTER_MOTOR_CHANNEL = 7;
    int HOPPER_MOTOR_CHANNEL = 5;
    
    // Digital IO Ports
    int SHOOTER_ENCODER_A_CHANNEL = 4;
    int SHOOTER_ENCODER_B_CHANNEL = 5;
    
	// Gamepad Ports
	int GAMEPAD_DRIVER_PORT = 0;
	
	// Gyro
	int DRIVETRAIN_GYRO_CHANNEL = 0;

	// Analog ports
    int HOPPER_SENSOR_CHANNEL = 3;
	
    // Hopper Constants
    double HOPPER_SENSOR_THRESHOLD = 1.25;

    
	// Physical Constants
    double DRIVETRAIN_WHEEL_CIRCUMFERENCE = 6 * Math.PI;
    double PULSES_PER_REVOLUTION = 250;
    double DRIVETRAIN_ENCODER_INCHES_PER_PULSE = DRIVETRAIN_WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;


}
