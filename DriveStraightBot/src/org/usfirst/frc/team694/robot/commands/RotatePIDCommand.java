package org.usfirst.frc.team694.robot.commands;

import org.usfirst.frc.team694.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotatePIDCommand extends PIDCommand {

	private double angle;
	private boolean auto;
	
	private static final double ANGLE_THRESHOLD = 2;
	private static final int TIME_WITHIN_THRESHOLD = 10;

	private int timeWithinCounter;

    public RotatePIDCommand(double angle) {
    	super(0.0,0.0,0.0);
		this.angle = angle;
    	requires(Robot.drivetrain);
    }

    public RotatePIDCommand() {
        this(0.0);
        auto = true;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (auto) {
            angle = SmartDashboard.getNumber("Rotate Angle",0.0);
        }

        PIDController controller = getPIDController();
		controller.setPID(
				SmartDashboard.getNumber("kP", 0.0),
				SmartDashboard.getNumber("kI", 0.0),
				SmartDashboard.getNumber("kD", 0.0)
				);
		controller.reset();
		if (!controller.isEnabled())
			controller.enable();

    	Robot.drivetrain.resetEncoders();
		Robot.drivetrain.gyroReset();
		Robot.drivetrain.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        // Threshold
    	if (Math.abs(Robot.drivetrain.gyroAngle() - angle) < ANGLE_THRESHOLD) {
    		timeWithinCounter++;
    	} else {
    		timeWithinCounter = 0;
    	}

    	// Print/Display
    	SmartDashboard.putNumber("Gyro Angle", Robot.drivetrain.gyroAngle());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (timeWithinCounter > TIME_WITHIN_THRESHOLD);
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("GYRO DISPLACEMENT: " + Robot.drivetrain.gyroAngle());
    	Robot.drivetrain.stop();
    	getPIDController().disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

	@Override
	protected double returnPIDInput() {
		// return offset
		return angle - Robot.drivetrain.gyroAngle();
	}

	@Override
	protected void usePIDOutput(double output) {
		// only rotate
		Robot.drivetrain.tankDrive(output, -output);
	}

}
