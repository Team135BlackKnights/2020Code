/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

	public RobotContainer container;
	Command autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	@Override
	public void robotInit() {
		container = new RobotContainer();

		// chooser.addOption("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
	}

	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	// Initialize Shutdown
	@Override
	public void disabledInit() {
		RobotContainer.turretLimelight.initLimelight(1, 0);
	}

	// Run Shutdown
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	// Initialize auto
	@Override
	public void autonomousInit() {
	}

	// Run auto
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	// Initialize Tele
	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	// Run Tele
	@Override
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
		// Scheduler.getInstance().run();

	}

	//
	@Override
	public void testPeriodic() {
	}
}
