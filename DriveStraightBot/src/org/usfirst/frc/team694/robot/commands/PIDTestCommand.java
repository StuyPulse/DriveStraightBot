package org.usfirst.frc.team694.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * PID Test: Drives straight, then drives back
 */
public class PIDTestCommand extends CommandGroup {

	private final double DISTANCE = 24.0; // inches
	private final double SPEED = 1.0;

    public PIDTestCommand() {
    	addSequential(new DriveStraightCommand(SPEED, DISTANCE));
    	addSequential(new DriveStraightCommand(-SPEED, DISTANCE));
    }
}
