package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private Joystick left = new Joystick(RobotMap.LEFT_JOYSTICK_USB);
    private Joystick right = new Joystick(RobotMap.RIGHT_JOYSTICK_USB);

    public double getLeftSpeed() {
	return left.getY();
    }

    public double getRightSpeed() {
	return right.getY();
    }
}
