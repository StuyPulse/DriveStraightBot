package org.usfirst.frc.team694.robot.commands;

import org.usfirst.frc.team694.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */

public class DriveStraightCommand extends Command {

	private double speed, distance;

    public DriveStraightCommand(double speed, double distance) {
    	this.speed = speed;
    	this.distance = distance;
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.resetEncoders();
    	Robot.drivetrain.startDrivingStraight(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("Encoders(L,R): (" 
    			+ Robot.drivetrain.leftEncoderDistance() + "," 
    			+ Robot.drivetrain.rightEncoderDistance() + "), gyro Angle: "
    			+ Robot.drivetrain.gyroAngle());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Robot.drivetrain.getEncoderDistance() > distance);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.stop();
    	Robot.drivetrain.disablePID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
