package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class PrintDriveData extends Command {

    private Robot robot = Robot.getInstance();

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
	double leftVel = robot.drivetrain.motionMagic.left.getVelocity();
	double rightVel = robot.drivetrain.motionMagic.right.getVelocity();
	double leftPos = robot.drivetrain.motionMagic.left.getPosition();
	double rightPos = robot.drivetrain.motionMagic.right.getPosition();

	System.out.printf("%s: joystick: %.3f, pos: %.2f, vel: %f%n", "Left", robot.oi.getLeftSpeed(), leftPos,
		leftVel);
	System.out.printf("%s: joystick: %.3f, pos: %.2f, vel: %f%n", "Right", robot.oi.getRightSpeed(), rightPos,
		rightVel);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
	return false;
    }

}
