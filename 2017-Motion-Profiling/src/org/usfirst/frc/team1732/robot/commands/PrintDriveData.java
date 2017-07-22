package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class PrintDriveData extends Command {

    private Robot robot = Robot.getInstance();

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
	System.out.printf("%s: joystick: %.3f, speed: %f%n", "Left", robot.oi.getLeftSpeed(),
		robot.drivetrain.motionMagic.left.getVelocity());
	System.out.printf("%s: joystick: %.3f, speed: %f%n", "Right", robot.oi.getRightSpeed(),
		robot.drivetrain.motionMagic.right.getVelocity());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
	return false;
    }

}
