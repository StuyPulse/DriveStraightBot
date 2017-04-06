package org.usfirst.frc.team694.robot.commands;

import org.usfirst.frc.team694.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShooterRegularStartCommand extends Command {

	private double speed;

    public ShooterRegularStartCommand(double speed) {
        requires(Robot.shooter);
        this.speed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Starting Shooter PID");
    	Robot.shooter.setSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("Shooter Speed", Robot.shooter.getCurrentMotorSpeedInRPM());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
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
