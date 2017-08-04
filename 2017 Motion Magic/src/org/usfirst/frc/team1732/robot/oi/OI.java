package org.usfirst.frc.team1732.robot.oi;

import org.usfirst.frc.team1732.robot.commands.joystickcommands.PrintDriveData;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the Robot.
 */
public class OI {

    public static final DriveController dualJoystick = new DriveController(2, 1, 0, 1, 1, false);
    public static final DriveController logitech = new DriveController(1, 0, 1, 1, 2, 3, 1, false);

    private DriveController controller;

    private JoystickButton printData;

    public OI(DriveController controller) {
	this.controller = controller;
	printData = controller.rightTriggerButton;
	printData.toggleWhenPressed(new PrintDriveData());
    }

    public DriveController controller() {
	return controller;
    }
}
