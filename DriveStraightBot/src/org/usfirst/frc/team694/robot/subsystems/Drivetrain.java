package org.usfirst.frc.team694.robot.subsystems;

import org.usfirst.frc.team694.robot.RobotMap;
import org.usfirst.frc.team694.robot.commands.DrivetrainTankDriveCommand;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drivetrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	/*
	private static double Kp;
	private static double Ki;
	private static double Kd;
	*/
	
	private PIDController controller;

	private DrivetrainOutput pidOutput;
	private GyroInput pidInput;

	private CANTalon leftFrontMotor;
	private CANTalon rightFrontMotor;
	private CANTalon leftBackMotor;
	private CANTalon rightBackMotor;
	private RobotDrive robotDrive;

	// replace with AHRS if necessary
	private Gyro gyro;
	
	// Movespeed for auto drive straight
	private double autoMoveSpeed = 0.0;

	public Drivetrain() {
		//TODO: Init Gyro gyro = new Gyro();
		pidOutput = new DrivetrainOutput(this);
		pidInput = new GyroInput(gyro);
		controller = new PIDController(0.0,0.0,0.0, pidInput, pidOutput);

		//super(Kp, Ki, Kd);
		leftFrontMotor = new CANTalon(RobotMap.LEFT_FRONT_MOTOR_PORT);
		rightFrontMotor = new CANTalon(RobotMap.RIGHT_FRONT_MOTOR_PORT);
		leftBackMotor = new CANTalon(RobotMap.LEFT_BACK_MOTOR_PORT);
		rightBackMotor = new CANTalon(RobotMap.RIGHT_BACK_MOTOR_PORT);
		//Assume the team does not have encoders if they do not have a mobility auton command.
		robotDrive = new RobotDrive(leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor);
	}

	public void tankDrive(double rightSpeed, double leftSpeed) {
		robotDrive.tankDrive(leftSpeed, rightSpeed);
	}

	public void stop() {
		leftFrontMotor.set(0);
		leftBackMotor.set(0);
		rightFrontMotor.set(0);
		rightBackMotor.set(0);
	}

	public void startDrivingStraight(double speed) {
		controller.setPID(
				SmartDashboard.getNumber("kP", 0.0),
				SmartDashboard.getNumber("kI", 0.0),
				SmartDashboard.getNumber("kD", 0.0)
				);
		autoMoveSpeed = speed;
		if (!controller.isEnabled()) {
			gyro.reset();
			controller.reset();
			controller.enable();
		}
	}
	
	public void disablePID() {
		controller.disable();
	}

	public void gyroReset() {
		gyro.reset();
	}
	
	public double gyroAngle() {
		return gyro.getAngle();
	}

	public void resetEncoders() {
        leftFrontMotor.reset();
        rightFrontMotor.reset();
        leftFrontMotor.enable();
        rightFrontMotor.enable();
        leftFrontMotor.setPosition(0);
        rightFrontMotor.setPosition(0);
    }

    public double leftEncoderDistance() {
        return (leftFrontMotor.getPosition() * RobotMap.DRIVETRAIN_ENCODERS_INCHES_PER_REVOLUTION)
                / RobotMap.DRIVETRAIN_ENCODERS_FACTOR;
    }

    public double rightEncoderDistance() {
        // Distance is scaled by -1.0 because right encoder was reporting
        // incorrect (negated) values
        return -1.0 * (rightFrontMotor.getPosition() * RobotMap.DRIVETRAIN_ENCODERS_INCHES_PER_REVOLUTION)
                / RobotMap.DRIVETRAIN_ENCODERS_FACTOR;
    }

	public double getEncoderDistance() {
        return Math.max(leftEncoderDistance(), rightEncoderDistance());	
	}

	protected void usePIDOutput(double output) {
		// these two (tankDrive and arcadeDrive for PID) 
		// are NOT compatible with each other, but as far as we know they are both viable
		robotDrive.tankDrive(autoMoveSpeed + output, autoMoveSpeed - output);
		//robotDrive.arcadeDrive(autoMoveSpeed, -output);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DrivetrainTankDriveCommand());
	}

	private class GyroInput implements PIDSource {

		private Gyro gyro;
		
		private PIDSourceType type;
		
		public GyroInput(Gyro gyro) {
			this.gyro = gyro;
			type = PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			return gyro.getAngle();
		}

		@Override
		public void setPIDSourceType(PIDSourceType type) {
			this.type = type;
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return type;
		}		
	}

	private class DrivetrainOutput implements PIDOutput {

		private Drivetrain drivetrain;
		
		public DrivetrainOutput(Drivetrain drivetrain) {
			this.drivetrain = drivetrain;
		}

		@Override
		public void pidWrite(double output) {
			drivetrain.usePIDOutput(output);	
		}

	}
}
