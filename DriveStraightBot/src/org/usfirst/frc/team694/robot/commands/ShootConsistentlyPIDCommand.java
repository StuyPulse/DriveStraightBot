package org.usfirst.frc.team694.robot.commands;

import org.usfirst.frc.team694.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShootConsistentlyPIDCommand extends PIDCommand {

	private double speed;
	private boolean auto;
	
    public ShootConsistentlyPIDCommand(double speed) {
    	super(0.0,0.0,0.0);
    	this.speed = speed;
        requires(Robot.shooter);
    }
    
    public ShootConsistentlyPIDCommand() {
    	this(0.0);
    	auto = true;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (auto) {
            speed = SmartDashboard.getNumber("Shooter Target Speed",0.0);
        }

        PIDController controller = getPIDController();
		controller.setPID(
				SmartDashboard.getNumber("Shooter P", 0.0),
				SmartDashboard.getNumber("Shooter I", 0.0),
				SmartDashboard.getNumber("Shooter D", 0.0)
				);
		controller.reset();
		if (!controller.isEnabled())
			controller.enable();

    	Robot.shooter.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Display/Testing
    	SmartDashboard.putNumber("Shooter Speed Offset", returnPIDInput());
    	SmartDashboard.putNumber("Shooter Speed", Robot.shooter.getCurrentMotorSpeedInRPM());
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

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		//return (Robot.shooter.getCurrentMotorSpeedInRPM() - speed);
		return Robot.shooter.getCurrentMotorSpeedInRPM();
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		//if (SmartDashboard.getBoolean("Use Shooter Voltage/RPM", false)) {
			Robot.shooter.setSpeed(speed + output);
		//} else {
		//	Robot.shooter.setRPM(output);
		//}
	}
}
