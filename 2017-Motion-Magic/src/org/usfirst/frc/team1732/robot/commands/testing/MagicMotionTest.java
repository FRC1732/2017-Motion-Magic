package org.usfirst.frc.team1732.robot.commands.testing;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

public class MagicMotionTest extends Command {

    private static boolean forward = true;
    private Robot robot = Robot.getInstance();

    public MagicMotionTest(double setpoint) {
	requires(robot.drivetrain);
	this.setpoint = forward ? setpoint : -setpoint;
	forward = !forward;
    }

    private double setpoint;

    private double acceleration = Drivetrain.DEFAULT_ACCELERATION;
    private double velocity = Drivetrain.DEFAULT_VELOCITY;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
	robot.drivetrain.changeToMotionMagic();
	robot.drivetrain.motionMagic.setMotionMagicAcceleration(acceleration);
	robot.drivetrain.motionMagic.setMotionMagicCruiseVelocity(velocity);
	robot.drivetrain.motionMagic.setSetpoint(setpoint);
	robot.drivetrain.motionMagic.resetPositions();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
	robot.drivetrain.motionMagic.graphData();
    }

    @Override
    protected boolean isFinished() {
	return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
	robot.drivetrain.changeToPercentVBus();
	robot.drivetrain.driveWithJoysticks(0, 0);
    }

}
