
package org.usfirst.frc.team1732.robot;

import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class Robot extends IterativeRobot {

    private static Robot robot;
    public RobotMap robotMap;
    public OI oi;
    public Drivetrain drivetrain;

    private Command autonomousCommand;

    public static Robot getInstance() {
	if (robot != null)
	    return robot;
	else {
	    System.err.println("VERY BAD ROBOT DOESN'T EXIST!");
	    return new Robot(); // this statement should never happen
	}
    }

    public Robot() {
	if (robot == null)
	    robot = this;
    }

    @Override
    public void robotInit() {
	robotMap = new RobotMap();
	oi = new OI();
	drivetrain = new Drivetrain();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
	Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
	autonomousCommand = null; // get selected command
	// schedule the autonomous command (example)
	if (autonomousCommand != null)
	    autonomousCommand.start();
    }

    @Override
    public void autonomousPeriodic() {
	Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
	if (autonomousCommand != null)
	    autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {
	Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() {
	LiveWindow.run();
    }
}
