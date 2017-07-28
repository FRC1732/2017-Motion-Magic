package org.usfirst.frc.team1732.robot.robotmaps;

/*
 * This class should contain abstract methods for differences between the practice bot/competition bot
 */
public abstract class RobotMap2017 extends RobotMap {

    @Override
    public int getPCM_CAN_ID() {
	return 0;
    }

    public int getLeftMasterMotorDeviceNumber() {
	return 5;
    }

    public int getLeft1MotorDeviceNumber() {
	return 6;
    }

    public int getLeft2MotorDeviceNumber() {
	return 7;
    }

    public int getRightMasterMotorDeviceNumber() {
	return 0;
    }

    public int getRight1MotorDeviceNumber() {
	return 1;
    }

    public int getRight2MotorDeviceNumber() {
	return 2;
    }

    public int getShifterSolenoidDeviceNumber() {
	return 0;
    }

}