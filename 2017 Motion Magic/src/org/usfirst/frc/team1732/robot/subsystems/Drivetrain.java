package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.commands.drive.TeleopDrive;
import org.usfirst.frc.team1732.robot.subsystems.motionmagic.MotionMagicPair;
import org.usfirst.frc.team1732.robot.subsystems.motionmagic.MotionMagicUnit;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {

    public static final String NAME = "Drive Train";

    public static final double ROBOT_WIDTH_INCHES = 26;
    public static final double ROBOT_LENGTH_INCHES = 34.5;
    public static final double TURNING_CIRCUMFERENCE = Math.PI * ROBOT_WIDTH_INCHES;
    public static final double EFFECTIVE_TURNING_CIRCUMFERENCE = TURNING_CIRCUMFERENCE * 1;

    // master is the motor that the other left motors follow
    private final CANTalon leftMaster = new CANTalon(Robot.RobotMap.getLeftMasterMotorDeviceNumber());
    private final CANTalon left1 = new CANTalon(Robot.RobotMap.getLeft1MotorDeviceNumber());
    private final CANTalon left2 = new CANTalon(Robot.RobotMap.getLeft2MotorDeviceNumber());
    // right motors
    // master is the motor the other right motors follow
    private final CANTalon rightMaster = new CANTalon(Robot.RobotMap.getRightMasterMotorDeviceNumber());
    private final CANTalon right1 = new CANTalon(Robot.RobotMap.getRight1MotorDeviceNumber());
    private final CANTalon right2 = new CANTalon(Robot.RobotMap.getRight2MotorDeviceNumber());

    private final Solenoid shifter = new Solenoid(Robot.RobotMap.getPCM_CAN_ID(),
	    Robot.RobotMap.getShifterSolenoidDeviceNumber());

    public static final boolean HIGH_GEAR = false; // false
    public static final boolean LOW_GEAR = !HIGH_GEAR;

    public final MotionMagicPair motionMagic;
    private MotionMagicUnit leftMM;
    private MotionMagicUnit rightMM;

    public static final double MAX_VOLTAGE = 12; // volts
    public static final int MAX_AMPS = 40;

    public static final int DEFAULT_PROFILE = 0;

    public static final int ENCODER_CODES_PER_REV = 1365; // = 4096 / 3
    // 4096 counts per encoder revolution, divided by 3 because of gearing
    public static final double INCHES_PER_REV = 3.473 * Math.PI;

    public static final boolean REVERSE_LEFT_ENCODER = true;
    public static final boolean REVERSE_RIGHT_ENCODER = false;

    public static final double MAX_ALLOWED_VELOCITY = 800; // RMP
    public static final double MAX_ALLOWED_ACCELERATION = 800; // RPM/sec

    public static final double MAX_LEFT_RPM = 830;
    public static final double MAX_RIGHT_RPM = 810;

    public static final CANTalon.VelocityMeasurementPeriod VELOCITY_MEASUREMENT_PERIOD = CANTalon.VelocityMeasurementPeriod.Period_1Ms;
    public static final int VELOCITY_MEASUREMENT_WINDOW = 10; // ms
    public static final int ALLOWED_ERROR = 0; // give in inches
    public static final int IZONE = 0; // give in inches
    public static final double CLOSED_LOOP_RAMP_RATE = MAX_VOLTAGE * 4;
    // ^ volts per second

    public static final double DEFAULT_ACCELERATION = 600; // RPM per second
    public static final double DEFAULT_VELOCITY = 600; // RPM

    public Drivetrain() {
	super(NAME);
	configureTalons();
	configureTalonEncoders();
	setBrakeMode(true);

	motionMagic = new MotionMagicPair(leftMM, rightMM);

	printData("Left Master", leftMaster);
	printData("Right Master", rightMaster);
    }

    private void configureTalons() {
	// reverses whole left side
	left1.changeControlMode(TalonControlMode.Follower);
	left1.set(leftMaster.getDeviceID());
	left2.changeControlMode(TalonControlMode.Follower);
	left2.set(leftMaster.getDeviceID());

	// sets right motors to follow right master
	rightMaster.reverseOutput(true);
	rightMaster.setInverted(true);
	right1.changeControlMode(TalonControlMode.Follower);
	right1.set(rightMaster.getDeviceID());
	right2.changeControlMode(TalonControlMode.Follower);
	right2.set(rightMaster.getDeviceID());

	leftMaster.setVoltageCompensationRampRate(MAX_VOLTAGE * 2);
	rightMaster.setVoltageCompensationRampRate(MAX_VOLTAGE * 2);
	// ^ apparently above units are volts per 100 ms
	shifter.set(HIGH_GEAR);
	setControlMode(TalonControlMode.PercentVbus);
    }

    private void configureTalonEncoders() {
	configureATalonEncoder(leftMaster, REVERSE_LEFT_ENCODER, DEFAULT_PROFILE);
	configureATalonEncoder(rightMaster, REVERSE_RIGHT_ENCODER, DEFAULT_PROFILE);

	setMotionMagicPID();

	leftMM = new MotionMagicUnit("Left", leftMaster, INCHES_PER_REV, ALLOWED_ERROR);
	rightMM = new MotionMagicUnit("Right", rightMaster, INCHES_PER_REV, ALLOWED_ERROR);
    }

    private void configureATalonEncoder(CANTalon talon, boolean reverseSensor, int defaultProfile) {
	talon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	talon.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 20);
	// talon.SetVelocityMeasurementPeriod(VELOCITY_MEASUREMENT_PERIOD);
	// talon.SetVelocityMeasurementWindow(VELOCITY_MEASUREMENT_WINDOW);
	int allowedError = (int) inchesToEncoderTicks(ALLOWED_ERROR);
	talon.setAllowableClosedLoopErr(allowedError);
	talon.configEncoderCodesPerRev(ENCODER_CODES_PER_REV);
	talon.reverseSensor(reverseSensor);
	talon.configNominalOutputVoltage(+0.0f, -0.0f);
	talon.configPeakOutputVoltage(+MAX_VOLTAGE, -MAX_VOLTAGE);
	// talon.setCurrentLimit(MAX_AMPS);
	// talon.EnableCurrentLimit(true);
	talon.setNominalClosedLoopVoltage(MAX_VOLTAGE);
	/*
	 * talon.setNominalClosedLoopVoltage does the same thing for closed loop
	 * as what voltage compensation mode does for teleop
	 */

	talon.setProfile(defaultProfile);
    }

    private void setMotionMagicPID() {
	double p = 0.1 * 1023 / 32_000 * 400;
	double i = 0;
	double d = p * 10 * 11;
	double fLeft = 1023 / (MAX_LEFT_RPM / 60.0 / 10.0 * ENCODER_CODES_PER_REV);
	double fRight = 1023 / (MAX_RIGHT_RPM / 60.0 / 10.0 * ENCODER_CODES_PER_REV);
	// might have different gains for each side
	leftMaster.setPID(p, i, d, fLeft, IZONE, CLOSED_LOOP_RAMP_RATE, DEFAULT_PROFILE);
	rightMaster.setPID(p, i, d, fRight, IZONE, CLOSED_LOOP_RAMP_RATE, DEFAULT_PROFILE);
    }

    public void setControlMode(TalonControlMode mode) {
	leftMaster.changeControlMode(mode);
	rightMaster.changeControlMode(mode);
    }

    public boolean isControlMode(TalonControlMode mode) {
	return leftMaster.getControlMode().equals(mode) && rightMaster.getControlMode().equals(mode);
    }

    public TalonControlMode getControlMode() {
	return leftMaster.getControlMode(); // both sides should be same
    }

    @Override
    public void initDefaultCommand() {
	setDefaultCommand(new TeleopDrive());
    }

    private double leftPercent;
    private double rightPercent;

    /*
     * Will only try to drive if in the correct mode
     */
    public void drive(double left, double right) {
	left = limit(left);
	right = limit(right);
	leftPercent = left;
	rightPercent = right;
	if (isControlMode(TalonControlMode.PercentVbus)) {
	    leftMaster.set(left);
	    rightMaster.set(right);
	} else if (isControlMode(TalonControlMode.Voltage)) {
	    leftMaster.set(left * MAX_VOLTAGE);
	    rightMaster.set(right * MAX_VOLTAGE);
	} else if (isControlMode(TalonControlMode.Speed)) {
	    leftMaster.set(left * MAX_ALLOWED_VELOCITY);
	    rightMaster.set(right * MAX_ALLOWED_VELOCITY);
	} else if (isControlMode(TalonControlMode.Current)) {
	    leftMaster.set(left * MAX_AMPS);
	    rightMaster.set(right * MAX_AMPS);
	}
    }

    private double limit(double d) {
	return d < 0 ? Math.max(d, -1) : Math.min(d, 1);
    }

    /**
     * @return the left percent voltage
     */
    public double getLeft() {
	return leftPercent;
    }

    /**
     * @return the right percent voltage
     */
    public double getRight() {
	return rightPercent;
    }

    public void setBrakeMode(boolean brake) {
	rightMaster.enableBrakeMode(brake);
	right1.enableBrakeMode(brake);
	right2.enableBrakeMode(brake);
	leftMaster.enableBrakeMode(brake);
	left1.enableBrakeMode(brake);
	left2.enableBrakeMode(brake);
    }

    public boolean isBrakingOn() {
	return rightMaster.getBrakeEnableDuringNeutral() && right1.getBrakeEnableDuringNeutral()
		&& right2.getBrakeEnableDuringNeutral() && leftMaster.getBrakeEnableDuringNeutral()
		&& left1.getBrakeEnableDuringNeutral() && left2.getBrakeEnableDuringNeutral();
    }

    private void printData(String name, CANTalon talon) {
	System.out.println(name + " firmware version: " + talon.GetFirmwareVersion());
	// System.out.println(name + " is encoder present? " +
	// talon.isSensorPresent(FeedbackDevice.QuadEncoder));
    }

    public static double encoderTicksToRev(double ticks) {
	return ticks / ENCODER_CODES_PER_REV;
    }

    public static double revToInches(double rev) {
	return INCHES_PER_REV * rev;
    }

    public static double inchesToRev(double inches) {
	return inches / INCHES_PER_REV;
    }

    public static double revToEncoderTicks(double rev) {
	return rev * ENCODER_CODES_PER_REV;
    }

    public static double encoderTicksToInches(double ticks) {
	return revToInches(encoderTicksToRev(ticks));
    }

    public static double inchesToEncoderTicks(double inches) {
	return revToEncoderTicks(inchesToRev(inches));
    }

}
