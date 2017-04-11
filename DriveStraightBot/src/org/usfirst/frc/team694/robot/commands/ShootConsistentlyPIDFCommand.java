package org.usfirst.frc.team694.robot.commands;

import org.usfirst.frc.team694.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShootConsistentlyPIDFCommand extends Command {

	private double speed;
	private boolean auto;
	
    public ShootConsistentlyPIDFCommand(double speed) {
    	this.speed = speed;
        requires(Robot.shooter);
    }
    
    public ShootConsistentlyPIDFCommand() {
    	this(0.0);
    	auto = true;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (auto) {
            speed = SmartDashboard.getNumber("Shooter Target Speed",0.0);
        }
        Robot.shooter.setPIDF(
				SmartDashboard.getNumber("Shooter P", 0.0),
				SmartDashboard.getNumber("Shooter I", 0.0),
				SmartDashboard.getNumber("Shooter D", 0.0),
				SmartDashboard.getNumber("Shooter F", 0.0)
        		);

    	Robot.shooter.setSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Display/Testing
    	double currentSpeed = Robot.shooter.getCurrentMotorSpeedInRPM();
    	SmartDashboard.putNumber("Shooter Speed Offset", currentSpeed - speed);
    	SmartDashboard.putNumber("Shooter Speed", currentSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// Stop when pressed DPad
        return Robot.oi.driverPad.getDPadDown().get();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
