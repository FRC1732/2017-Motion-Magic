package org.usfirst.frc.team1732.robot.commands.testing;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveTimeTest extends Command {

    private Robot robot = Robot.getInstance();
    private static boolean forward = true;
    private double left;
    private double right;

    public DriveTimeTest(double sec, double speed) {
	this(sec, speed, speed);
    }

    public DriveTimeTest(double sec, double leftSpeed, double rightSpeed) {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	requires(robot.drivetrain);
	setTimeout(sec);
	int sign = forward ? 1 : -1;
	left = leftSpeed * sign;
	right = rightSpeed * sign;
	forward = !forward;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
	robot.drivetrain.changeToPercentVBus();
	robot.drivetrain.driveWithJoysticks(left, right);
    }

    private double leftMax = 0;
    private double rightMax = 0;

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
	double d;
	double leftVelocity = robot.drivetrain.motionMagic.left.getVelocity();
	if (Math.abs((d = leftVelocity)) > Math.abs(leftMax)) {
	    leftMax = d;
	}
	double rightVelocity = robot.drivetrain.motionMagic.right.getVelocity();
	if (Math.abs((d = rightVelocity)) > Math.abs(rightMax)) {
	    rightMax = d;
	}
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
	return isTimedOut();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
	System.out.println("Drive Time Max Velocities: ");
	System.out.println("Left Max: " + leftMax);
	System.out.println("Right Max: " + rightMax);
	robot.drivetrain.driveWithJoysticks(0, 0);
    }

}
