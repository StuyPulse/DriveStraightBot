package org.usfirst.frc.team694.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * PID Test: Drives straight, then drives back
 */
public class PIDTestCommand extends CommandGroup {

	//private final double DISTANCE = 1.0; // inches
	//private final double SPEED = 0.1;

    public PIDTestCommand() {
    	addSequential(new DriveStraightPIDCommand());
        //addSequential(new DriveStraightPIDCommand(SPEED, DISTANCE));
    	//addSequential(new DriveStraightPIDCommand(-SPEED, DISTANCE));
    }
}
