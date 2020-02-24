/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Turret;

public class rotateAndTiltTurretToPos extends CommandBase {
  /**
   * Creates a new rotateAndTiltTurretToPos.
   */
  private Turret turret;
  private double rotationPos, tiltPos;
  private boolean isFinished;
  public rotateAndTiltTurretToPos(Turret _turret, double _rotationPos, double _tiltPos) 
  {
    turret = _turret;
    rotationPos = _rotationPos;
    tiltPos = _tiltPos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isFinished = false; 
    SmartDashboard.putString("Turret Command Running: ", "rotate and tilt turret to pos" + rotationPos + tiltPos);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double currentTiltPos, currentRotationPos; 
    currentTiltPos = turret.getSparkEncoderPosition(turret.tiltEncoder);
    currentRotationPos = turret.getSparkEncoderPosition(turret.rotationEncoder);

    double tiltError, rotationError;
    tiltError = tiltPos-currentTiltPos;
    rotationError = rotationPos - currentRotationPos;

    double tiltPower, rotationPower;
    tiltPower =  0;
    rotationPower = 0;
    //Sometimes all you need is BANG BANG control; 
    if(Math.abs(tiltError) < 5 && Math.abs(rotationError) < 5)
    {
      if(tiltPos < currentTiltPos)
      {
        tiltPower = .65;
      }
      else if(tiltPos > currentTiltPos)
      {
        tiltPower = -.65;
      }
      if(rotationPos < currentRotationPos)
      {
        rotationPower = -.85;
      }
      else if(rotationPos > currentRotationPos)
      {
        rotationPower = .85;
      }
      isFinished = false;
    }
    else 
    {
      tiltPower = 0;
      rotationPower = 0;
      isFinished = true;
    }

    turret.aimTurret(rotationPower, tiltPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    turret.aimTurret(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
