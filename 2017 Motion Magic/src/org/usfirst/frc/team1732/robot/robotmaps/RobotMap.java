package org.usfirst.frc.team1732.robot.robotmaps;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public abstract class RobotMap {

    public abstract int getPCM_CAN_ID();

}
