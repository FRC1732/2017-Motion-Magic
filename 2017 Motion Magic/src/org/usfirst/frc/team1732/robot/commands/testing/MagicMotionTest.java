package org.usfirst.frc.team1732.robot.commands.testing;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class MagicMotionTest extends Command {

    private static boolean forward = true;

    public MagicMotionTest(double setpoint) {
	requires(Robot.drivetrain);
	this.setpoint = forward ? setpoint : -setpoint;
	forward = !forward;
    }

    private double setpoint;

    private double acceleration = Drivetrain.DEFAULT_ACCELERATION;
    private double velocity = Drivetrain.DEFAULT_VELOCITY;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
	Robot.drivetrain.setControlMode(TalonControlMode.MotionMagic);
	Robot.drivetrain.motionMagic.setMotionMagicAcceleration(acceleration);
	Robot.drivetrain.motionMagic.setMotionMagicCruiseVelocity(velocity);
	Robot.drivetrain.motionMagic.setSetpoint(setpoint);
	Robot.drivetrain.motionMagic.resetPositions();
	Robot.drivetrain.updateMotionMagicPID();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
	Robot.drivetrain.motionMagic.graphData();
    }

    @Override
    protected boolean isFinished() {
	return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
	Robot.drivetrain.setControlMode(TalonControlMode.Voltage);
	Robot.drivetrain.drive(0, 0);
    }

}
