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

public class shootXBalls extends CommandBase {
  /**
   * Creates a new shootXBalls.
   */
  Turret turret;
  int ballsToShoot, ballsShot, initialBallsShot;
  boolean isFinished, isReadyToShoot; 
  double errorSum;

  public shootXBalls(Turret _turret, int _ballsToShoot) {
    turret = _turret;
    ballsToShoot = _ballsToShoot;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    turret.initLimelight(0, 0);
    initialBallsShot = turret.ballsShot;
    errorSum = 0;
    SmartDashboard.putString("New Turret Command Running: ", "shoot x balls x = " + ballsToShoot) ;
    isReadyToShoot = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.updateBallCount();

    ballsShot = turret.ballsShot - initialBallsShot;
    SmartDashboard.putNumber("shoot x balls shot", ballsShot);
    isFinished = (ballsShot >= ballsToShoot);
    
    double desiredRPM, actualRPM, rpmError, rpmFeedForward, kF, kP, shooterInput,
    horizontalOffset, distanceToTarget, rP, hP, minRotationError, rotationError, rotationInput, minHoodError, hoodError, hoodDesired, hoodActual, hoodInput; 
    boolean isTargetWithinRange, isTargetValid, isShooting;

    distanceToTarget = turret.getAreaDistance();
    SmartDashboard.putNumber("distance to Target", distanceToTarget);
    horizontalOffset = turret.limelightData[1] - 3.5;
    isTargetValid = turret.limelightData[0] >= 1;

    minRotationError = .025;
    minHoodError = 2;
    isTargetValid = turret.getLimelightData()[0] >= 1;

    rotationError = horizontalOffset / 30 ;
    hoodDesired = (-1.1855* Math.pow(distanceToTarget, 2)) + (22.115 * distanceToTarget) + 78.0045;
    SmartDashboard.putNumber("Desired Hood Pos", hoodDesired);
    hoodActual = turret.getHoodPos();
    hoodError = hoodDesired - hoodActual;
    actualRPM = turret.getShooterVel();
    isShooting = actualRPM > 2000;

    isTargetWithinRange = ((isTargetValid && Math.abs(rotationError) < minRotationError && Math.abs(hoodError) < minHoodError) || isShooting);


    if(!isTargetWithinRange)
    {
      desiredRPM = 1700;
    }
    else 
    {
      desiredRPM = (20.9988 * Math.pow(distanceToTarget, 2))  + (122.8191* distanceToTarget) + 2285.4107;
    }
    rpmError = (desiredRPM - actualRPM)/6000;

    rpmFeedForward = desiredRPM / turret.maxRPM;


    if(isTargetWithinRange)
    {
      isReadyToShoot = true; 
    }

    turret.isReadyForBall = (!(desiredRPM == 1700) && Math.abs(rpmError) <= .075);

    rP = 1.2;

    hP = .1;
    kF = .625;
    kP = 2;
    



    rotationInput = rotationError * rP;
    hoodInput = hoodError * hP;
    shooterInput = kF * rpmFeedForward + kP * rpmError;
    
    if (turret.isReadyForBall) {
      turret.runIndexer(.3);
    } else {
      turret.runIndexer(0);
    }

    if(!isTargetWithinRange)
    {
      turret.aimTurret(rotationInput, hoodInput);
    }
    else 
    {
      turret.aimTurret(0, 0);
    }

    if (isReadyToShoot && ballsShot >= 0 ) {
      turret.runShooter(shooterInput);
    } 
    else if(!isTargetWithinRange)
    {
      turret.runShooter(shooterInput);
    }
    else 
    {
      turret.runShooter(0);

    }
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopAllMotors();
    turret.isReadyForBall = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
