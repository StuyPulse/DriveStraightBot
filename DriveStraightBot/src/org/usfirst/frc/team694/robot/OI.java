package org.usfirst.frc.team694.robot;

import org.usfirst.frc.team694.robot.commands.HopperRunCommand;
import org.usfirst.frc.team694.robot.commands.ShootConsistentlyPIDCommand;
import org.usfirst.frc.team694.robot.commands.ShootConsistentlyPIDFCommand;
import org.usfirst.frc.team694.robot.commands.ShooterRegularStartCommand;
import org.usfirst.frc.team694.robot.commands.ShooterRegularStopCommand;
import org.usfirst.frc.team694.util.Gamepad;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public Gamepad driverPad = new Gamepad(RobotMap.GAMEPAD_DRIVER_PORT);

	public OI() {
		// TODO: Uncomment on
		//driverPad.getLeftButton().whenPressed(new PIDDriveStraightTestCommand());
		//driverPad.getRightButton().whenPressed(new PIDRotateTestCommand());
		//driverPad.getTopButton().whenPressed(new UselessFPSCounterCommand(60));

		driverPad.getRightButton().whenPressed(new ShootConsistentlyPIDFCommand());
		driverPad.getBottomButton().whenPressed(new ShootConsistentlyPIDCommand());
		driverPad.getDPadRight().whenPressed(new ShooterRegularStartCommand(0.5));
		driverPad.getDPadDown().whenPressed(new ShooterRegularStopCommand());
		
        driverPad.getLeftTrigger().whileHeld(new HopperRunCommand(true));
        driverPad.getLeftBumper().whileHeld(new HopperRunCommand(false));

	}
}
