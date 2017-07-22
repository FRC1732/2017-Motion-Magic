package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ToggleDriveMode extends Command {

    private Robot robot = Robot.getInstance();

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
	if (robot.isEnabled() && robot.isOperatorControl())
	    robot.drivetrain.changeToVoltageMode();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
	return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
	if (robot.isEnabled() && robot.isOperatorControl())
	    robot.drivetrain.changeToPercentVBus();
    }

}
