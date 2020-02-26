/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

	public RobotContainer container;
	private Command autoCommand;
	public String autoSelected;
	SendableChooser<Command> chooser = new SendableChooser<>();

	@Override
	public void robotInit() {
		container = new RobotContainer();
		SmartDashboard.putString("Auto Selection", "Default");
		String autoSelected = SmartDashboard.getString("Auto Selection", "Default");
		switch(autoSelected)
		{
			case "Right Side" : autoCommand = container.getRightSideAuto();
			break;
			case "Middle" : autoCommand = container.getMiddleAuto();
			break; 
			case "Auto Line" : autoCommand = container.getAutoLine();
			break;
			case "Auto Line+" : autoCommand = container.getDefaultAuto();
			break;
		}

		chooser.addOption("Auto Line", container.getAutoLine());
		chooser.addOption("Right Side", container.getRightSideAuto());
		chooser.addOption("Middle", container.getMiddleAuto());
		chooser.addOption("Auto Line+", container.getDefaultAuto());
		SmartDashboard.putString("Auto Selected", chooser.toString());
	
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
		RobotContainer.turretLimelight.initLimelight(0, 0);
		autoCommand = container.getAutonomousCommand();
		
		if(autoCommand != null)
		{
			autoCommand.schedule();
		}
	}

	// Run auto
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	// Initialize Tele
	@Override
	public void teleopInit() {
		if (autoCommand != null) {
			autoCommand.cancel();
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
