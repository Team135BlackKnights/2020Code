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
import frc.robot.RobotMap;

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
    // This method will be called once per scheduler run
  }
}
