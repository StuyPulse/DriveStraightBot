package org.usfirst.frc.team694.robot;

import org.usfirst.frc.team694.robot.commands.PIDTestCommand;
import org.usfirst.frc.team694.util.Gamepad;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	Gamepad driverpad = new Gamepad(RobotMap.GAMEPAD_DRIVER_PORT);

	public OI() {
		driverpad.getLeftButton().whenPressed(new PIDTestCommand());
	}
}
