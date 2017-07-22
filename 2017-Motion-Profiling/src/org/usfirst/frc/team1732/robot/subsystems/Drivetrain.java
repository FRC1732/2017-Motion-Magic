package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.RobotMap;
import org.usfirst.frc.team1732.robot.commands.DriveWithJoysticks;

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

    public static final double MAX_VOLTAGE = 12;

    public static final int DEFAULT_PROFILE = 0;

    public static final int ENCODER_CODES_PER_REV = 1365; // = 4096 / 3
    // 4096 counts per encoder revolution, divided by 3 because of gearing
    public static final double INCHES_PER_REV = 3.911 * Math.PI;

    public static final boolean REVERSE_LEFT_ENCODER = true;
    public static final boolean REVERSE_RIGHT_ENCODER = false;

    public static final double MAX_LEFT_RPM = 0;
    public static final double MAX_RIGHT_RPM = 0;

    public static final CANTalon.VelocityMeasurementPeriod VELOCITY_MEASUREMENT_PERIOD = CANTalon.VelocityMeasurementPeriod.Period_1Ms;
    public static final int VELOCITY_MEASUREMENT_WINDOW = 10; // ms
    public static final int ALLOWED_ERROR = 3; // give in inches
    public static final double CLOSED_LOOP_RAMP_RATE = 1023 * 1.0;
    // need to check what units CLOSED_LOOP_RAMP_RATE is in (I think 0 - 2013);
    public static final int IZONE = 0;

    public static final double DEFAULT_ACCELERATION = 0;
    public static final double DEFAULT_VELOCITY = 0;

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

	leftMaster.setVoltageCompensationRampRate(24.0);
	rightMaster.setVoltageCompensationRampRate(24.0);

	changeToPercentVBus();
    }

    private void configureTalonEncoders() {
	leftMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	rightMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);

	leftMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 20);
	rightMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 20);

	// leftMaster.SetVelocityMeasurementPeriod(VELOCITY_MEASUREMENT_PERIOD);
	// rightMaster.SetVelocityMeasurementPeriod(VELOCITY_MEASUREMENT_PERIOD);
	//
	// leftMaster.SetVelocityMeasurementWindow(VELOCITY_MEASUREMENT_WINDOW);
	// rightMaster.SetVelocityMeasurementWindow(VELOCITY_MEASUREMENT_WINDOW);
	//
	// leftMaster.setAllowableClosedLoopErr(ALLOWED_ERROR / INCHES_PER_REV);
	// rightMaster.setAllowableClosedLoopErr(ALLOWED_ERROR /
	// INCHES_PER_REV);

	leftMaster.configEncoderCodesPerRev(ENCODER_CODES_PER_REV);
	rightMaster.configEncoderCodesPerRev(ENCODER_CODES_PER_REV);

	leftMaster.reverseSensor(REVERSE_LEFT_ENCODER);
	rightMaster.reverseSensor(REVERSE_RIGHT_ENCODER);

	leftMaster.configNominalOutputVoltage(+0.0f, -0.0f);
	rightMaster.configNominalOutputVoltage(+0.0f, -0.0f);

	leftMaster.configPeakOutputVoltage(+12.0f, -12.0f);
	rightMaster.configPeakOutputVoltage(+12.0f, -12.0f);

	leftMaster.setProfile(DEFAULT_PROFILE);
	rightMaster.setProfile(DEFAULT_PROFILE);

	setMotionMagicPID();

	leftMM = new MotionMagicUnit("Left", leftMaster, INCHES_PER_REV, ALLOWED_ERROR);
	rightMM = new MotionMagicUnit("Right", rightMaster, INCHES_PER_REV, ALLOWED_ERROR);
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
	System.out.println(name + " is encoder present? " + talon.isSensorPresent(FeedbackDevice.QuadEncoder));
    }

}
