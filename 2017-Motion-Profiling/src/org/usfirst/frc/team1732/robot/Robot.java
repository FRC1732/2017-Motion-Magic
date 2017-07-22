
package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class Robot extends IterativeRobot {

    public static OI oi;

    private Command autonomousCommand;

    @Override
    public void robotInit() {
	oi = new OI();
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
