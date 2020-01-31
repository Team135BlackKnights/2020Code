/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.*;

public class ShootTurret extends CommandBase {
  /**
   * Creates a new ShootTurret.
   */
  private final Turret turret;
  private final DoubleSupplier topShooterSupplier, bottomShooterSupplier;
  public boolean isFinished;

  public ShootTurret(Turret subsystem, DoubleSupplier topShooterRPM, DoubleSupplier bottomShooterRPM) 
  {
    turret = subsystem;
    topShooterSupplier = topShooterRPM;
    bottomShooterSupplier = bottomShooterRPM;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    SmartDashboard.putString("Turret Command Running:","Shoot Turret");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double _topShooterRPM = topShooterSupplier.getAsDouble();
    double _bottomShooterRPM = bottomShooterSupplier.getAsDouble();
    if(turret.isBallInShooter())
    {
     turret.runShooterRPM(_topShooterRPM, _bottomShooterRPM);
     turret.runBallFeeder(0);
      isFinished = false;
    }
    else if(turret.isBallInShooter() &&turret.turretBallCount == 0)
    {
     turret.runBallFeeder(.75);
      isFinished = false;
    }
    else if(turret.isBallInShooter() &&turret.turretBallCount >= 1)
    {
     turret.turretBallCount--;
      isFinished = true;
    }
    else if(!turret.isBallInShooter() &&turret.turretBallCount >=1)
    {
     turret.runBallFeeder(.75);
     turret.turretBallCount--;
      isFinished = false;
    }
    else {
     turret.runShooterRPM(_topShooterRPM, _bottomShooterRPM);
     turret.runBallFeeder(.75);
    }
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
