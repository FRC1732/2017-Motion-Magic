package org.usfirst.frc.team1732.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class DriveController {

    private final int leftUSB;
    private final int leftX;
    private final int leftY;
    private final int leftTrigger;

    private final int rightUSB;
    private final int rightX;
    private final int rightY;
    private final int rightTrigger;

    public final boolean isTriggerAxis;

    public final Joystick leftStick;
    public final Joystick rightStick;

    public final JoystickButton leftTriggerButton;
    public final JoystickButton rightTriggerButton;

    /**
     * Constructor meant to use with the dual joysticks <br>
     * Assumes that both joysticks have same axis numbers
     * 
     * @param leftUSB
     *            left USB number
     * @param rightUSB
     *            right USB number
     * @param xAxis
     *            joystick x axis number
     * @param yAxis
     *            joystick y axis number
     * @param trigger
     *            trigger axis/button number
     * @param isTriggerAxis
     *            if the trigger is an axis (range of values) or just a button
     *            (on and off)
     */
    public DriveController(int leftUSB, int rightUSB, int xAxis, int yAxis, int trigger, boolean isTriggerAxis) {
	this(leftUSB, rightUSB, xAxis, yAxis, trigger, xAxis, yAxis, trigger, isTriggerAxis);
    }

    /**
     * Constructor meant for game controllers
     * 
     * @param USB
     *            the single USB port for the controller
     * @param leftX
     *            left x axis number
     * @param leftY
     *            left y axis number
     * @param leftTrigger
     *            left trigger axis/button number
     * @param rightX
     *            right x axis number
     * @param rightY
     *            right x axis number
     * @param rightTrigger
     *            right trigger axis/button number
     * @param isTriggerAxis
     *            if the trigger is an axis (range of values) or just a button
     *            (on and off)
     */
    public DriveController(int USB, int leftX, int leftY, int leftTrigger, int rightX, int rightY, int rightTrigger,
	    boolean isTriggerAxis) {
	this(USB, USB, leftX, leftY, leftTrigger, rightX, rightY, rightTrigger, isTriggerAxis);
    }

    /**
     * Constructor with all arguments. Generally one of the other two
     * constructors should be used.
     * 
     * @param leftUSB
     *            left USB number
     * @param rightUSB
     *            right USB number
     * @param leftX
     *            left x axis number
     * @param leftY
     *            left y axis number
     * @param leftTrigger
     *            left trigger axis/button number
     * @param rightX
     *            right x axis number
     * @param rightY
     *            right x axis number
     * @param rightTrigger
     *            right trigger axis/button number
     * @param isTriggerAxis
     *            if the trigger is an axis (range of values) or just a button
     *            (on and off)
     */
    public DriveController(int leftUSB, int rightUSB, int leftX, int leftY, int leftTrigger, int rightX, int rightY,
	    int rightTrigger, boolean isTriggerAxis) {
	this.leftUSB = leftUSB;
	this.leftX = leftX;
	this.leftY = leftY;
	this.leftTrigger = leftTrigger;
	this.rightUSB = rightUSB;
	this.rightX = rightX;
	this.rightY = rightY;
	this.rightTrigger = rightTrigger;
	this.isTriggerAxis = isTriggerAxis;

	leftStick = new Joystick(leftUSB);
	rightStick = new Joystick(rightUSB);

	leftTriggerButton = new JoystickButton(leftStick, leftTrigger);
	rightTriggerButton = new JoystickButton(rightStick, rightTrigger);
    }

    public double getLeftX() {
	return leftStick.getRawAxis(leftX);
    }

    public double getLeftY() {
	return leftStick.getRawAxis(leftY);
    }

    public double getLeftTrigger() {
	if (isTriggerAxis) {
	    return leftStick.getRawAxis(leftTrigger);
	} else {
	    return 0;
	}
    }

    public double getRightX() {
	return rightStick.getRawAxis(rightX);
    }

    public double getRightY() {
	return rightStick.getRawAxis(rightY);
    }

    public double getRightTrigger() {
	if (isTriggerAxis) {
	    return rightStick.getRawAxis(rightTrigger);
	} else {
	    return 0;
	}
    }

}