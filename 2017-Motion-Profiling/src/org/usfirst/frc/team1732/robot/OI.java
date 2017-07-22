package org.usfirst.frc.team1732.robot;

import org.usfirst.frc.team1732.robot.commands.ToggleDriveMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private RobotMap robotMap = Robot.getInstance().robotMap;

    private Joystick left = new Joystick(robotMap.LEFT_JOYSTICK_USB);
    private Joystick right = new Joystick(robotMap.RIGHT_JOYSTICK_USB);

    private final int trigger = 1;
    private JoystickButton drivingMode = new JoystickButton(left, trigger);

    public OI() {
	drivingMode.toggleWhenActive(new ToggleDriveMode());
    }

    public double getLeftSpeed() {
	double l = left.getY();
	System.out.println("Left Joystick: " + l);
	return l;
    }

    public double getRightSpeed() {
	double r = right.getY();
	System.out.println("Right Joystick: " + r);
	return r;
    }
}
