/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.*;

public class ShootTurret extends CommandBase {

  private final Turret turret;
  private final double topShooterDesired, bottomShooterDesired;
  private boolean isFinished;

  public ShootTurret(Turret subsystem, double topShooterRPM, double bottomShooterRPM) 
  {
    turret = subsystem;
    topShooterDesired = topShooterRPM;
    bottomShooterDesired = bottomShooterRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    SmartDashboard.putString("Turret Command Running:","Shoot Turret ");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feederMax = 4500;

    double feederDesired = -.25*feederMax;

    double topShooterActual = turret.getTopWheelRPM();
    double bottomShooterActual = turret.getBottomWheelRPM();
    double feederActual = turret.getFeederRPM();
  
    double topShooterError = topShooterDesired-topShooterActual;
    double bottomShooterError = bottomShooterDesired-bottomShooterActual;
    double feederError = feederDesired-feederActual;


    double topPower = topShooterDesired+topShooterError;
    double bottomPower = bottomShooterDesired+bottomShooterError;
    double feederPower = feederDesired+feederError;

    topPower = topPower/5600;
    bottomPower = bottomPower/5200;
    feederPower = feederPower/ feederMax;


    double tP, bP, fP; 

    tP = 1;
    bP = 1;
    fP = 1; 


    double topInput = topPower*tP;
    double bottomInput = bottomPower*bP;
    double feederInput = feederPower*fP;

    turret.runShooterPower(topInput, bottomInput);
    turret.runBallFeeder(feederInput);
    //turret.runBallFeeder(1);
    /*
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
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.runShooterPower(0, 0);
    turret.runBallFeeder(0);
    SmartDashboard.putString("Turret Command Running:", "No Command Running");
    SmartDashboard.putString("Shoot Turret: ", "Has Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
