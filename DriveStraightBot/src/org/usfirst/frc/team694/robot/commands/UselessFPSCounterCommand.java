package org.usfirst.frc.team694.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */

// To figure out what the fps is for the robot. We could look this up but I'm tired of not knowing
public class UselessFPSCounterCommand extends Command {

	private int framesToCount;
	
	private int counter;

	private double startTime;

    public UselessFPSCounterCommand(int framesToCount) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.framesToCount = framesToCount;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	counter = 0;
    	startTime = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	counter++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (counter >= framesToCount);
    }

    // Called once after isFinished returns true
    protected void end() {
    	double dt = Timer.getFPGATimestamp() - startTime;
    	double fps = counter / dt;
    	System.out.println("TIME TO COUNT " + counter + " FRAMES: " + dt + ", FPS: " + fps);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
