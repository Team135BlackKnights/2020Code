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
import frc.robot.subsystems.*;


public class Robot extends TimedRobot {
  //Dec FalconDrivers with drive
  public static FalconDrive drive; 
  //Declare ColorWheel with colorwheel
  public static ColorWheel colorWheel;
  //Dec Prototyping with prototyping
  public static Prototyping prototyping;
  //Dec TurretLimelight with turretlimelight
  public static TurretLimelight  turretlimelight;
  //Dec IntakeLimelight with intakelimelight
  public static IntakeLimelight intakeLimelight; 
  //Dec Turret with turret
  public static Turret turret;
  //Dec OI with oi
  public static OI oi;

  Command autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  @Override
  public void robotInit() {
	SmartDashboard.putNumber("ENABLED", 1);
	//Cam Drivers
	drive = FalconDrive.getInstance();
	//Camera Sensors for Turret (Outer/Inner Circle)
	turretlimelight = TurretLimelight.getInstance();
	//Camera Sensors for Intake (# of balls through intake)
	intakeLimelight = IntakeLimelight.getInstance();

	prototyping = Prototyping.getInstance();

	colorWheel = ColorWheel.getInstance();
	//turret = Turret.getInstance();
	oi = new OI();


    
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

    //Initialize Shutdown
 	@Override
	public void disabledInit() {}

	//Run Shutdown
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	//Initialize auto
	@Override
	public void autonomousInit() {}

	//Run auto
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	//Initialize Tele
	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	//Run Tele
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
	}

	//
	@Override
	public void testPeriodic() {}
}
