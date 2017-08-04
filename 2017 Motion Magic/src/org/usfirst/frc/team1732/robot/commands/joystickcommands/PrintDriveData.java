package org.usfirst.frc.team1732.robot.commands.joystickcommands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class PrintDriveData extends Command {

    protected void init() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
	double left = Robot.drivetrain.getLeft();
	double right = Robot.drivetrain.getRight();
	double leftPos = Robot.drivetrain.motionMagic.left.getPosition();
	double rightPos = Robot.drivetrain.motionMagic.right.getPosition();
	double leftVel = Robot.drivetrain.motionMagic.left.getVelocity();
	double rightVel = Robot.drivetrain.motionMagic.right.getVelocity();

	print("Left", left, leftPos, leftVel);
	print("Right", right, rightPos, rightVel);
    }

    private void print(String name, double percentVoltage, double pos, double vel) {
	System.out.printf("%s: joystick: %.3f, pos: %.2f, vel: %f%n", name, percentVoltage, pos, vel);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
	return false;
    }

}
