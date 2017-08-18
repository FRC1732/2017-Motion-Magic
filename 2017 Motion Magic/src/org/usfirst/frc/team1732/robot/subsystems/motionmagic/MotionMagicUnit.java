package org.usfirst.frc.team1732.robot.subsystems.motionmagic;

import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionMagicUnit {

    private final String name;
    private CANTalon talon;
    private final double inchesPerRotation;
    private final double allowedError;

    /**
     * Constructs a MagicMotionUnit wrapper object
     * 
     * @param t
     *            the CANTalon object to be used. It should already be
     *            configured
     */
    public MotionMagicUnit(String name, CANTalon talon, double inchesPerRotation, double allowedError) {
	this.name = name;
	this.talon = talon;
	this.inchesPerRotation = inchesPerRotation;
	this.allowedError = allowedError;
    }

    private boolean isInMotionMagicMode() {
	return talon.getControlMode().equals(TalonControlMode.MotionMagic);
    }

    public void resetPosition() {
	talon.setPosition(0);
    }

    public void setMotionMagicCruiseVelocity(double velocity) {
	talon.setMotionMagicCruiseVelocity(velocity);
    }

    public void setMotionMagicAcceleration(double acceleration) {
	talon.setMotionMagicAcceleration(acceleration);
    }

    /**
     * Sets the setpoint for the magic motion to drive to<br>
     * Only works if currently in MotionMagic Mode
     * 
     * @param inches
     *            setpoint in inches
     */
    public void setSetpoint(double inches) {
	if (isInMotionMagicMode()) {
	    talon.set(inches / inchesPerRotation);
	}
    }

    public boolean onTarget() {
	return Math.abs(talon.getClosedLoopError()) < allowedError / inchesPerRotation;
    }

    public double getVelocity() {
	return talon.getSpeed();
    }

    public double getPosition() {
	return talon.getPosition();
    }

    private int prints = 0;

    public void graphData() {
	double vel = talon.getSpeed();
	double position = talon.getPosition();
	double throttle = (talon.getOutputVoltage() / 12.0) * 1023;
	double current = talon.getOutputCurrent();
	double error = talon.getClosedLoopError();
	double errorRotations = Drivetrain.encoderTicksToRev(error);
	double setpoint = talon.getSetpoint();
	if (prints % 10 == 0) {
	    String s = String.format(
		    "%s: Vel: %.3f, Position: %.2f, Setpoint: %.2f, Error: %.2f, Error (Rotations): %.2f, Throttle: %.2f, ",
		    name, vel, position, setpoint, error, errorRotations, throttle);
	    System.out.println(s);
	    prints = 0;
	} else {
	    prints++;
	}
	// for graphing

	SmartDashboard.putNumber(name + " Vel (RPM)", vel);
	// SmartDashboard.putNumber(name + " Vel (inches)", vel *
	// inchesPerRotation);
	SmartDashboard.putNumber(name + " Pos (revs)", position);
	SmartDashboard.putNumber(name + " Pos (inches)", Drivetrain.revToInches(position));
	// SmartDashboard.putNumber(name + " Pos (inches)", position *
	// inchesPerRotation);
	SmartDashboard.putNumber(name + " Setpoint (revs)", setpoint);
	SmartDashboard.putNumber(name + " ClosedLoopError", error);
	SmartDashboard.putNumber(name + " AppliedThrottle", throttle);
	SmartDashboard.putNumber(name + " Current", current);
    }
}