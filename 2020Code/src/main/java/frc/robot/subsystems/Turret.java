/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.*;

public class Turret extends Subsystem implements RobotMap.MOTORS { 
  /**
   * Creates a new Turret.
   */
  public static Turret instance; 
  public TalonSRX turretTalon;
  public static Turret getInstance() { if (instance == null) {instance = new Turret(); } return instance;}
  private Turret() 
  {
    turretTalon = new TalonSRX(TURRET_TALON_ID);
    turretTalon.configReverseSoftLimitEnable(false);
    turretTalon.configForwardSoftLimitEnable(false);

  }
  public void rotateTurret(double power)
  {
     turretTalon.set(ControlMode.PercentOutput, power);
  }
  public void stopTurret()
  {
    turretTalon.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void initDefaultCommand() 
  {
  }
  @Override
  public void periodic() 
  {
    rotateTurret(-.75);
    /*
    if(Robot.limelight.GetLimelightData()[1] > 1 && Robot.limelight.GetLimelightData()[0] >= 1)
		{
			rotateTurret(.75);
			SmartDashboard.putString("Direction turret turning:", "left");
      SmartDashboard.putString("Turret Not moving", "false");

    }
	else if (Robot.limelight.GetLimelightData()[1] < 1 && Robot.limelight.GetLimelightData()[0] >= 1)
		{
			rotateTurret(-.75);
			SmartDashboard.putString("Direction turret turning:", "right");
      SmartDashboard.putString("Turret Not moving", "false");

		}
		else {
      rotateTurret(1);
      SmartDashboard.putString("Turret Not moving", "true");
    }
    */
    // This method will be called once per scheduler run
  }
}
