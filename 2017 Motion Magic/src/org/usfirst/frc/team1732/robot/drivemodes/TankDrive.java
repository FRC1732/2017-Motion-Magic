package org.usfirst.frc.team1732.robot.drivemodes;

import java.util.function.Function;

import org.usfirst.frc.team1732.robot.oi.DriveController;

public class TankDrive extends DriveMode {

    private Function<Double, Double> inputOutputMapper;

    public TankDrive(DriveController controller) {
	/*
	 * input -> input does same thing as in this comment
	 * 
	 * Function<Double, Double> inputOutputMapper = (input) -> { return
	 * input; };
	 */
	this(controller, input -> input);
    }

    public TankDrive(DriveController controller, Function<Double, Double> inputOutputMapper) {
	super(controller);
	this.inputOutputMapper = inputOutputMapper;
    }

    @Override
    public double getLeftOutput() {
	return inputOutputMapper.apply(controller.getLeftY());
    }

    @Override
    public double getRightOutput() {
	return inputOutputMapper.apply(controller.getRightY());
    }

}
