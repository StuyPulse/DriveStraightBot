package org.usfirst.frc.team694.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PIDRotateTestCommand extends CommandGroup {

    public PIDRotateTestCommand() {
    	addSequential(new RotatePIDCommand());
    }
}
