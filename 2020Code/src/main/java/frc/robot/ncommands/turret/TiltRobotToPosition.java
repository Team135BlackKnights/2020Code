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

public class TiltRobotToPosition extends CommandBase {
  private final Turret turret;
  private final double _targetPos;
  private boolean isFinished = false;
  public TiltRobotToPosition(Turret subsystem, double targetPos) {
    turret = subsystem;
    this._targetPos = targetPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Turret Commmand Running: ", "tiltRobotToPosition" + _targetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTurretPos = turret.getSparkEncoderPosition(turret.tiltEncoder);
    double positionError = currentTurretPos-_targetPos;
    SmartDashboard.putNumber("rotate turret position erro", positionError);
    double tolerance = 5;
    double tiltPower = 0;
    if (Math.abs(this._targetPos-currentTurretPos) > 5){
      if(_targetPos <currentTurretPos)
      {
        tiltPower = -.85;
      }
      else if(_targetPos >currentTurretPos)
      {
        tiltPower = .85;
      }
      isFinished = false;
    } else {
      tiltPower = 0; 
      isFinished = true;
    }
    turret.runRotation(tiltPower);
    SmartDashboard.putBoolean("Should be done rotating", isFinished);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
