package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.RobotMap;
import org.usfirst.frc.team1732.robot.commands.drive.DriveWithJoysticks;
import org.usfirst.frc.team1732.robot.subsystems.motionmagic.MotionMagicPair;
import org.usfirst.frc.team1732.robot.subsystems.motionmagic.MotionMagicUnit;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {

    private final RobotMap robotMap = Robot.getInstance().robotMap;

    public static final String NAME = "Drive Train";

    public static final double ROBOT_WIDTH_INCHES = 26;
    public static final double ROBOT_LENGTH_INCHES = 34.5;
    public static final double TURNING_CIRCUMFERENCE = Math.PI * ROBOT_WIDTH_INCHES;
    public static final double EFFECTIVE_TURNING_CIRCUMFERENCE = TURNING_CIRCUMFERENCE * 1;

    // master is the motor that the other left motors follow
    private final CANTalon leftMaster = new CANTalon(robotMap.LEFT_MASTER_MOTOR_DEVICE_NUMBER);
    private final CANTalon left1 = new CANTalon(robotMap.LEFT_1_MOTOR_DEVICE_NUMBER);
    private final CANTalon left2 = new CANTalon(robotMap.LEFT_2_MOTOR_DEVICE_NUMBER);
    // right motors
    // master is the motor the other right motors follow
    private final CANTalon rightMaster = new CANTalon(robotMap.RIGHT_MASTER_MOTOR_DEVICE_NUMBER);
    private final CANTalon right1 = new CANTalon(robotMap.RIGHT_1_MOTOR_DEVICE_NUMBER);
    private final CANTalon right2 = new CANTalon(robotMap.RIGHT_2_MOTOR_DEVICE_NUMBER);

    public final MotionMagicPair motionMagic;
    private MotionMagicUnit leftMM;
    private MotionMagicUnit rightMM;

    public static final double MAX_VOLTAGE = 12; // volts

    public static final int DEFAULT_PROFILE = 0;

    public static final int ENCODER_CODES_PER_REV = 1365; // = 4096 / 3
    // 4096 counts per encoder revolution, divided by 3 because of gearing
    public static final double INCHES_PER_REV = 3.911 * Math.PI;

    public static final boolean REVERSE_LEFT_ENCODER = true;
    public static final boolean REVERSE_RIGHT_ENCODER = false;

    public static final double MAX_ALLOWED_VELOCITY = 0; // RMP
    public static final double MAX_ALLOWED_ACCELERATION = 0; // RMP/sec

    public static final double MAX_LEFT_RPM = 0;
    public static final double MAX_RIGHT_RPM = 0;

    public static final CANTalon.VelocityMeasurementPeriod VELOCITY_MEASUREMENT_PERIOD = CANTalon.VelocityMeasurementPeriod.Period_1Ms;
    public static final int VELOCITY_MEASUREMENT_WINDOW = 10; // ms
    public static final int ALLOWED_ERROR = 0; // give in inches
    public static final int IZONE = 0; // give in inches
    public static final double CLOSED_LOOP_RAMP_RATE = MAX_VOLTAGE * 4;
    // ^ volts per second

    public static final double DEFAULT_ACCELERATION = 0; // RPM per second
    public static final double DEFAULT_VELOCITY = 0; // RPM

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
	leftMaster.setInverted(true);
	left1.changeControlMode(TalonControlMode.Follower);
	left1.set(leftMaster.getDeviceID());
	left2.changeControlMode(TalonControlMode.Follower);
	left2.set(leftMaster.getDeviceID());

	// sets right motors to follow right master
	right1.changeControlMode(TalonControlMode.Follower);
	right1.set(rightMaster.getDeviceID());
	right2.changeControlMode(TalonControlMode.Follower);
	right2.set(rightMaster.getDeviceID());

	leftMaster.setVoltageCompensationRampRate(MAX_VOLTAGE * 2);
	rightMaster.setVoltageCompensationRampRate(MAX_VOLTAGE * 2);
	// ^ apparently above units are volts per 100 ms

	changeToPercentVBus();
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
	talon.setNominalClosedLoopVoltage(MAX_VOLTAGE);
	// this ^ does the same thing for closed loop as what voltage
	// compensation mode does for teleop

	talon.setProfile(defaultProfile);
    }

    private void setMotionMagicPID() {
	double p, i, d, f;
	// might have different gains for each side
	leftMaster.setPID(p = 0, i = 0, d = 0, f = 0, IZONE, CLOSED_LOOP_RAMP_RATE, DEFAULT_PROFILE);
	rightMaster.setPID(p = 0, i = 0, d = 0, f = 0, IZONE, CLOSED_LOOP_RAMP_RATE, DEFAULT_PROFILE);
    }

    public void changeToMotionMagic() {
	leftMaster.changeControlMode(TalonControlMode.MotionMagic);
	rightMaster.changeControlMode(TalonControlMode.MotionMagic);
    }

    public void changeToPercentVBus() {
	leftMaster.changeControlMode(TalonControlMode.PercentVbus);
	rightMaster.changeControlMode(TalonControlMode.PercentVbus);
    }

    public void changeToVoltageMode() {
	leftMaster.changeControlMode(TalonControlMode.Voltage);
	rightMaster.changeControlMode(TalonControlMode.Voltage);
    }

    public boolean isInMotionMagicMode() {
	return leftMaster.getControlMode().equals(TalonControlMode.MotionMagic)
		&& rightMaster.getControlMode().equals(TalonControlMode.MotionMagic);
    }

    public boolean isInPercentVBusMode() {
	return leftMaster.getControlMode().equals(TalonControlMode.PercentVbus)
		&& rightMaster.getControlMode().equals(TalonControlMode.PercentVbus);
    }

    public boolean isInVoltageMode() {
	return leftMaster.getControlMode().equals(TalonControlMode.Voltage)
		&& rightMaster.getControlMode().equals(TalonControlMode.Voltage);
    }

    @Override
    public void initDefaultCommand() {
	setDefaultCommand(new DriveWithJoysticks());
    }

    public void driveWithJoysticks(double left, double right) {
	tankDrive(left, right);
    }

    /*
     * Will only try to drive if in the correct mode
     */
    private void tankDrive(double left, double right) {
	if (isInPercentVBusMode()) {
	    leftMaster.set(left);
	    rightMaster.set(right);
	} else if (isInVoltageMode()) {
	    leftMaster.set(left * MAX_VOLTAGE);
	    rightMaster.set(right * MAX_VOLTAGE);
	}
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

    private double encoderTicksToRev(double ticks) {
	return ticks / ENCODER_CODES_PER_REV;
    }

    private double revToInches(double rev) {
	return INCHES_PER_REV * rev;
    }

    private double inchesToRev(double inches) {
	return inches / INCHES_PER_REV;
    }

    private double revToEncoderTicks(double rev) {
	return rev * ENCODER_CODES_PER_REV;
    }

    private double encoderTicksToInches(double ticks) {
	return revToInches(encoderTicksToRev(ticks));
    }

    private double inchesToEncoderTicks(double inches) {
	return revToEncoderTicks(inchesToRev(inches));
    }

}
