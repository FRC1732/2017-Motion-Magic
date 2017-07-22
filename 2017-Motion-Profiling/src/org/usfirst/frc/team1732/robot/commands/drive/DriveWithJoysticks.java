package org.usfirst.frc.team1732.robot.commands.drive;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoysticks extends Command {

    private Robot robot = Robot.getInstance();

    public DriveWithJoysticks() {
	super("Drive With Joysticks");
	requires(robot.drivetrain);
    }

    @Override
    public void initialize() {
	if (robot.drivetrain.isInMotionMagicMode())
	    robot.drivetrain.changeToPercentVBus();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
	double left = robot.oi.getLeftSpeed();
	double right = robot.oi.getRightSpeed();
	robot.drivetrain.driveWithJoysticks(left, right);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
	return false;
    }
}
