package org.usfirst.frc.team694.robot.commands;

import org.usfirst.frc.team694.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */

public class DriveStraightPIDCommand extends Command {

	private double speed, distance;
	private boolean auto;
	
    public DriveStraightPIDCommand(double speed, double distance) {
    	this.speed = speed;
    	this.distance = distance;
        requires(Robot.drivetrain);
    }
    
    public DriveStraightPIDCommand() {
        this(0.0,0.0);
        auto = true;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (auto) {
            speed = SmartDashboard.getNumber("Drive Speed",0.0);
            distance = SmartDashboard.getNumber("Drive Distance",0.0);
        }
        
        Robot.drivetrain.gyroReset();

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
        return (Robot.drivetrain.getAbsEncoderDistance() > distance);
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("GYRO DISPLACEMENT: " + Robot.drivetrain.gyroAngle());
    	Robot.drivetrain.stop();
    	Robot.drivetrain.disablePID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
