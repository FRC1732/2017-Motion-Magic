package org.usfirst.frc.team1732.robot.drivemodes;

import org.usfirst.frc.team1732.robot.oi.DriveController;

/**
 * Super class for implementing different drive modes. <br>
 * Only meant for skid-steer type Robots
 */
public abstract class DriveMode {

    protected final DriveController controller;

    public DriveMode(DriveController controller) {
	this.controller = controller;
    }

    public abstract double getLeftOutput();

    public abstract double getRightOutput();

}