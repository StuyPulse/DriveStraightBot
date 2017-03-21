package org.usfirst.frc.team694.robot.commands;

import org.usfirst.frc.team694.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraightCommand extends Command {

    private double speed;
    private double distance;
    private boolean auto;
    
    public DriveStraightCommand(double speed, double distance) {
        // Use requires() here to declare subsystem dependencies
        this.speed = speed;
        this.distance = distance;
        requires(Robot.drivetrain);
    }

    public DriveStraightCommand() {
        this(0.0,0.0);
        auto = true;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (auto) {
            speed = SmartDashboard.getNumber("Drive Speed",0.0);
            distance = SmartDashboard.getNumber("Drive Distance",0.0);
        }
        System.out.println("speed: " + speed + ", distance: " + distance);
        Robot.drivetrain.resetEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.drivetrain.tankDrive(speed,speed);
        //System.out.println("distance: " + Robot.drivetrain.getEncoderDistance());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Robot.drivetrain.getAbsEncoderDistance() > distance || distance <= 0.01 || speed <= 0.01);
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
