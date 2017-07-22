package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

public class DriveArc extends Command {

    private final Robot robot = Robot.getInstance();

    private final double distance;
    private final double radius;
    private final double velocity;
    private final double acceleration;
    private final boolean goLeft;

    public DriveArc(double distance, double radius, double velocity, double acceleration, boolean goLeft) {
	super();
	this.distance = distance;
	this.radius = radius;
	this.velocity = Math.min(velocity, Drivetrain.MAX_ALLOWED_VELOCITY);
	this.acceleration = Math.min(acceleration, Drivetrain.MAX_ALLOWED_ACCELERATION);
	this.goLeft = goLeft;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
	double outerRadius = radius + Drivetrain.ROBOT_WIDTH_INCHES / 2.0;
	double innerRadius = radius - Drivetrain.ROBOT_WIDTH_INCHES / 2.0;

	double percentDriven = distance / circumference(radius);
	double outerDistance = percentDriven * circumference(outerRadius);
	double innerDistance = percentDriven * circumference(innerRadius);

	double outerVelocity = velocity;
	double outerAcceleration = acceleration;

	// calculate these numbers so that both sides have the same angular
	// velocity throughout the turn

	double innerVelocity = 1;
	double innerAcceleration = 1;

	robot.drivetrain.changeToMotionMagic();
	if (goLeft) {
	    robot.drivetrain.motionMagic.left.setSetpoint(innerDistance);
	    robot.drivetrain.motionMagic.left.setMotionMagicCruiseVelocity(innerVelocity);
	    robot.drivetrain.motionMagic.left.setMotionMagicAcceleration(innerAcceleration);

	    robot.drivetrain.motionMagic.right.setSetpoint(outerDistance);
	    robot.drivetrain.motionMagic.right.setMotionMagicCruiseVelocity(outerVelocity);
	    robot.drivetrain.motionMagic.right.setMotionMagicAcceleration(outerAcceleration);
	} else {
	    robot.drivetrain.motionMagic.right.setSetpoint(innerDistance);
	    robot.drivetrain.motionMagic.right.setMotionMagicCruiseVelocity(innerVelocity);
	    robot.drivetrain.motionMagic.right.setMotionMagicAcceleration(innerAcceleration);

	    robot.drivetrain.motionMagic.left.setSetpoint(outerDistance);
	    robot.drivetrain.motionMagic.left.setMotionMagicCruiseVelocity(outerVelocity);
	    robot.drivetrain.motionMagic.left.setMotionMagicAcceleration(outerAcceleration);
	}
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
	return robot.drivetrain.motionMagic.onTarget();
    }

    private double circumference(double r) {
	return Math.PI * r * 2;
    }
}
