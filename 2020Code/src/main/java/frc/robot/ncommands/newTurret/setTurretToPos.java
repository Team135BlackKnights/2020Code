/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.newTurret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.newTurret;

public class setTurretToPos extends CommandBase {
  /**
   * Creates a new setTurretToPos.
   */
  private newTurret turret;
  private double rotationPos, hoodPos;
  private boolean isFinished;
  public setTurretToPos(newTurret _turret, double _rotationPos, double _hoodPos)
  {
    turret = _turret;
    rotationPos = _rotationPos;
    hoodPos = _hoodPos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isFinished = false;
    turret.resetAllTurretEncoders();
    SmartDashboard.putString("New Turret Command Running: ", "set Turret To Pos");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    double currentHoodPos, currentRotationPos, hoodPower, rotationPower, rotationError, hoodError;
    currentHoodPos = turret.getHoodPos();
    currentRotationPos = turret.getRotationPos();

    hoodError = hoodPos - currentHoodPos;
    rotationError = rotationPos - currentRotationPos;

    isFinished = (Math.abs(hoodError) <=5 && Math.abs(rotationError) <= 5);
    if(hoodError > 5)
    {
      hoodPower = -.45;
    }
    else if(hoodError < 5)
    {
      hoodPower = .45;
    }
    else 
    {
      hoodPower = 0;
    }

    if(rotationError > 5)
    {
      rotationPower = -.45;
    }
    else if (rotationError < 5)
    {
      rotationPower = .45;
    }
    else 
    {
      rotationPower = 0;
    }

    turret.aimTurret(rotationPower, hoodPower);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    turret.stopTurret();
    SmartDashboard.putString("New Turret Command Running: ", "No Command Running");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
