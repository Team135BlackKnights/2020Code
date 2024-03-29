/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.RobotContainer.activeBallCount;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

	public RobotContainer RobotContainer;
	private Command autoCommand;
	SendableChooser<Command> autoChooser = new SendableChooser<>();

	@Override
	public void robotInit() {

		RobotContainer = new RobotContainer();
		RobotContainer.initLimelight(2, 0);
		autoChooser.setDefaultOption("Auto Line Plus", RobotContainer.getAutoLinePlus());
		autoChooser.addOption("Auto Line", RobotContainer.getAutoLine());
		//autoChooser.addOption("Right Side ", RobotContainer.getRightSideAuto());

		//SmartDashboard.putData(autoChooser);

	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	// Initialize Shutdown
	@Override
	public void disabledInit() {
		RobotContainer.initLimelight(1, 0);
	}

	// Run Shutdown
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		autoCommand = autoChooser.getSelected();
		//SmartDashboard.putString("Auto Command ", autoChooser.getSelected().toString());
	}

	// Initialize auto
	@Override
	public void autonomousInit() {
		RobotContainer.initLimelight(0, 0);
		frc.robot.RobotContainer.drive.resetEncoders();
		activeBallCount = 3;
		autoCommand = autoChooser.getSelected();
		SmartDashboard.putString("Auto Command ", autoChooser.getSelected().toString());
		if (autoCommand != null) {
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
		frc.robot.RobotContainer.activeBallCount = 0;
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
