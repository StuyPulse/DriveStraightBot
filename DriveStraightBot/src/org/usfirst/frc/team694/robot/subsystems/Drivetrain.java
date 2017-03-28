package org.usfirst.frc.team694.robot.subsystems;

import org.usfirst.frc.team694.robot.RobotMap;
import org.usfirst.frc.team694.robot.commands.DrivetrainTankDriveCommand;
import org.usfirst.frc.team694.util.StuyGyro;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drivetrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.


	private CANTalon leftFrontMotor;
	private CANTalon rightFrontMotor;
	private CANTalon leftBackMotor;
	private CANTalon rightBackMotor;
	private RobotDrive robotDrive;
	
    private Encoder leftEncoder;
    private Encoder rightEncoder;


	// replace with AHRS if necessary
	private StuyGyro gyro;
	

	public Drivetrain() {
        gyro = new StuyGyro(RobotMap.DRIVETRAIN_GYRO_CHANNEL);
//		controller = new PIDController(0.0,0.0,0.0, pidInput, pidOutput);

		leftFrontMotor = new CANTalon(RobotMap.LEFT_FRONT_MOTOR_PORT);
		rightFrontMotor = new CANTalon(RobotMap.RIGHT_FRONT_MOTOR_PORT);
		leftBackMotor = new CANTalon(RobotMap.LEFT_BACK_MOTOR_PORT);
		rightBackMotor = new CANTalon(RobotMap.RIGHT_BACK_MOTOR_PORT);
		
		// This shouldn't be needed but whatever
		leftFrontMotor.enableBrakeMode(true);
        rightFrontMotor.enableBrakeMode(true);
        leftBackMotor.enableBrakeMode(true);
        rightBackMotor.enableBrakeMode(true);

        leftEncoder = new Encoder(RobotMap.DRIVETRAIN_ENCODER_LEFT_CHANNEL_A, RobotMap.DRIVETRAIN_ENCODER_LEFT_CHANNEL_B, true, EncodingType.k2X);
        leftEncoder.setDistancePerPulse(RobotMap.DRIVETRAIN_ENCODER_INCHES_PER_PULSE);
        rightEncoder = new Encoder(RobotMap.DRIVETRAIN_ENCODER_RIGHT_CHANNEL_A, RobotMap.DRIVETRAIN_ENCODER_RIGHT_CHANNEL_B, false, EncodingType.k2X);
        rightEncoder.setDistancePerPulse(RobotMap.DRIVETRAIN_ENCODER_INCHES_PER_PULSE);


		//Assume the team does not have encoders if they do not have a mobility auton command.
		robotDrive = new RobotDrive(leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor);
		
		stop();
		gyroReset();
	}

	public void tankDrive(double leftSpeed, double rightSpeed) {
	    SmartDashboard.putNumber("Gyro Angle", gyroAngle());
	    SmartDashboard.putNumber("Left Encoder", leftEncoderDistance());
        SmartDashboard.putNumber("Right Encoder", rightEncoderDistance());

	    robotDrive.tankDrive(leftSpeed, rightSpeed);
	}

	public void stop() {
		leftFrontMotor.set(0);
		leftBackMotor.set(0);
		rightFrontMotor.set(0);
		rightBackMotor.set(0);
	}

	public void gyroReset() {
		gyro.reset();
		gyro.resetGyroMeasurements();
	}

	public double gyroAngle() {
		return gyro.getInstantaneousGyroAngleInDegrees();
	}

	public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getAbsEncoderDistance() {
        return Math.max(Math.abs(leftEncoderDistance()), Math.abs(rightEncoderDistance()));
    }

    public double leftEncoderDistance() {
        return leftEncoder.getDistance();
    }

    public double rightEncoderDistance() {
        return rightEncoder.getDistance();
    }

	public double getEncoderDistance() {
        return Math.max(leftEncoderDistance(), rightEncoderDistance());	
	}


	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DrivetrainTankDriveCommand());
	}

}
