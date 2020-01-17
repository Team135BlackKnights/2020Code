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
  public static FalconDrive drive; 
  public static ColorWheel colorWheel;
  public static Prototyping prototyping;
  public static TurretLimelight  turretlimelight;
  public static IntakeLimelight intakeLimelight; 
  public static Turret turret;
  public static OI oi;

  Command autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  @Override
  public void robotInit() {
	
	drive = FalconDrive.getInstance();

	turretlimelight = TurretLimelight.getInstance();
	intakeLimelight = IntakeLimelight.getInstance();
	prototyping = Prototyping.getInstance();

	//colorWheel = ColorWheel.getInstance();
	//turret = Turret.getInstance();
	oi = new OI();


    
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  @Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
	}

	@Override
	public void testPeriodic() {}
}
