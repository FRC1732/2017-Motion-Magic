package org.usfirst.frc.team1732.robot.commands.drive;

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
	this.radius = Math.abs(radius);
	this.velocity = Math.min(Math.abs(velocity), Drivetrain.MAX_ALLOWED_VELOCITY);
	this.acceleration = Math.min(Math.abs(acceleration), Drivetrain.MAX_ALLOWED_ACCELERATION);
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

	if (radius < Drivetrain.ROBOT_WIDTH_INCHES / 2.0) {
	    innerDistance = -innerDistance;
	}

	double outerVelocity = velocity;
	double outerAcceleration = acceleration;

	/*
	 * velocity and acceleration don't need to be converted to inch/min and
	 * inch/min/sec even though the circumference is inches because later
	 * they would be converted back to RPM and RPM/sec anyways
	 */
	double rotationalCruiseVelocity = outerVelocity / circumference(outerRadius);
	double rotationalAcceleration = outerAcceleration / circumference(outerRadius);

	// unused but I think having these here improves understanding
	double angularCruiseVelocity = rotationalCruiseVelocity * 2 * Math.PI;
	double angularAcceleration = rotationalAcceleration * 2 * Math.PI;

	double innerVelocity = rotationalCruiseVelocity * circumference(innerRadius);
	double innerAcceleration = rotationalAcceleration * circumference(innerRadius);

	/*
	 * theoretically, if we did the limit as radius approached infinity all
	 * the infinite radius stuff would cancel out and this if block wouldn't
	 * be needed, but java doesn't work like that
	 */
	if (radius == Double.POSITIVE_INFINITY) {
	    outerRadius = radius;
	    innerRadius = radius;
	    innerDistance = distance;
	    outerDistance = distance;
	    innerVelocity = outerVelocity;
	    innerAcceleration = outerAcceleration;
	}

	if (goLeft) {
	    robot.drivetrain.motionMagic.left.setMotionMagicCruiseVelocity(innerVelocity);
	    robot.drivetrain.motionMagic.left.setMotionMagicAcceleration(innerAcceleration);

	    robot.drivetrain.motionMagic.right.setMotionMagicCruiseVelocity(outerVelocity);
	    robot.drivetrain.motionMagic.right.setMotionMagicAcceleration(outerAcceleration);

	    robot.drivetrain.motionMagic.left.setSetpoint(innerDistance);
	    robot.drivetrain.motionMagic.right.setSetpoint(outerDistance);
	} else {
	    robot.drivetrain.motionMagic.right.setMotionMagicCruiseVelocity(innerVelocity);
	    robot.drivetrain.motionMagic.right.setMotionMagicAcceleration(innerAcceleration);

	    robot.drivetrain.motionMagic.left.setSetpoint(outerDistance);
	    robot.drivetrain.motionMagic.left.setMotionMagicCruiseVelocity(outerVelocity);

	    robot.drivetrain.motionMagic.right.setSetpoint(innerDistance);
	    robot.drivetrain.motionMagic.left.setMotionMagicAcceleration(outerAcceleration);
	}
	robot.drivetrain.motionMagic.resetPositions();
	robot.drivetrain.changeToMotionMagic();
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
