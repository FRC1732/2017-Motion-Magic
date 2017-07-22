package org.usfirst.frc.team1732.robot.subsystems.motionmagic;

public class MotionMagicPair {

    public final MotionMagicUnit left;
    public final MotionMagicUnit right;

    public MotionMagicPair(MotionMagicUnit l, MotionMagicUnit r) {
	left = l;
	right = r;
    }

    public void resetPositions() {
	left.resetPosition();
	right.resetPosition();
    }

    public void setMotionMagicCruiseVelocity(double velocity) {
	left.setMotionMagicCruiseVelocity(velocity);
	right.setMotionMagicCruiseVelocity(velocity);
    }

    public void setMotionMagicAcceleration(double acceleration) {
	left.setMotionMagicAcceleration(acceleration);
	right.setMotionMagicAcceleration(acceleration);
    }

    /**
     * Sets the setpoint for the magic motion to drive to<br>
     * Only works if currently in MotionMagic Mode
     * 
     * @param inches
     *            setpoint in inches
     */
    public void setSetpoint(double inches) {
	left.setSetpoint(inches);
	right.setSetpoint(inches);
    }

    public boolean onTarget() {
	return left.onTarget() && right.onTarget();
    }

    public void graphData() {
	left.graphData();
	right.graphData();
    }

}