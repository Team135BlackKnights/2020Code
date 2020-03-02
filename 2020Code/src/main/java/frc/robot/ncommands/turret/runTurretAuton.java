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
import frc.robot.util.MovingAverage;
import frc.robot.*;

public class runTurretAuton extends CommandBase {
  /**
   * Creates a new runTurretAuton.
   */
  private Turret turret;
  private TurretLimelight limelight;
  private Storage storage;
  boolean isFinished, turretReadyToRetarget, hasShot3Balls;
  double topWheelErrorSum, bottomWheelErrorSum, ballsBeforeDone;
  boolean hasShotAtLeastOnce;
  int count;
  
  public runTurretAuton(Turret _turret, TurretLimelight _limelight, Storage _storage, double balls) 
  {
    turret = _turret;
    limelight = _limelight;
    storage = _storage;
    ballsBeforeDone = balls;
    addRequirements(turret, limelight, storage);
    
    //testp
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    SmartDashboard.putString("Turret Command Running: ", "runTurretAuton");
    SmartDashboard.putString("Storage Command Running: ", "runTurretAuton");
    limelight.initLimelight(0, 0);
    topWheelErrorSum = 0;
    bottomWheelErrorSum = 0;
    turret.turretBallCount = 0; 
    count = 0;
    isFinished = false;
    turretReadyToRetarget = true;
    hasShotAtLeastOnce = false;
    hasShot3Balls = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    isFinished = turret.turretBallCount >= ballsBeforeDone;
    hasShot3Balls=(turret.turretBallCount >=3);
    
    if(hasShot3Balls)
    {
      turret.turretBallCount = 0;
      ballsBeforeDone = ballsBeforeDone-3;
    }
    boolean targetExist, isDriving, isTargetWithinRange, isShooterUpToSpeed;
    double verticalOffset, horizontalOffset, 
    distanceToTarget, rotationPower, tiltPower, currentTopWheelRPM, currentBottomWheelRPM, 
    desiredTopWheelRPM, desiredBottomWheelRPM, feederDesired, feederMax, feederActual,
    topShooterMax, bottomShooterMax, storageMax, storageDesired, storageActual, 
    feederError, tiltError, rotationError, storageError, topWheelError, bottomWheelError, 
    topWheelInput, bottomWheelInput, feederInput, storageInput, rotationInput, tiltInput,
    currentStoragePos;

    targetExist = limelight.GetLimelightData()[0] >=1;
    distanceToTarget = limelight.distToTarget() ;
    verticalOffset = limelight.GetLimelightData()[2] +distanceToTarget/3.5;
    horizontalOffset = limelight.GetLimelightData()[1] - distanceToTarget/4.4;
    
    currentTopWheelRPM = turret.getTopWheelRPM();
    currentBottomWheelRPM = turret.getBottomWheelRPM();
    feederActual = turret.getFeederRPM();
    storageActual = storage.getConveyorVel();
    currentStoragePos = storage.getConveyorRotations();
    MovingAverage smoothStorage = new MovingAverage(25);
    double averagedStorage = smoothStorage.process((float)storageActual);
    
    isDriving = Math.abs(RobotContainer.drive.getLinearMps()) >.05;
    isTargetWithinRange = (Math.abs(verticalOffset) < .75 && Math.abs(horizontalOffset) < .75);
    if(!targetExist)
    {
      isTargetWithinRange = true;
    }

    feederMax = 5000;
    topShooterMax = 5100;
    bottomShooterMax = 5000;
    storageMax = 5200;

    desiredTopWheelRPM = 1400;

    if(Math.abs(RobotContainer.drive.getLeftMetres()) > 1.9)
    {
      desiredTopWheelRPM = -18.526*Math.pow(distanceToTarget,2) + 258.5128*distanceToTarget + 1667.3675;
    }  

    SmartDashboard.putNumber("desired auto", desiredTopWheelRPM);
    desiredBottomWheelRPM = desiredTopWheelRPM * 1.25;
    feederDesired = .35*feederMax;
    storageDesired = -1500;
    
    topWheelError = (desiredTopWheelRPM - currentTopWheelRPM)/topShooterMax;
    bottomWheelError = (desiredBottomWheelRPM - currentBottomWheelRPM)/bottomShooterMax;
    feederError = feederDesired - feederActual;
    storageError = storageDesired-averagedStorage;
    storageError = storageError/storageMax;
  
    double tP, bP, fP, tI, bI, rP, rI, tiltP, storageP, 
    rotationErrorSum, fF, bF, tF;

    rP = 1.4; rI = .25; tiltP = .87; storageP = 2.1;
    bI = 2; tI = 2.5; tP = .5; bP = .94; fP = .862;
    tF = .85; bF = .85; fF = .892;

    rotationErrorSum =+ horizontalOffset*.02;

    topWheelErrorSum = topWheelErrorSum + tI*topWheelError*.02;
    bottomWheelErrorSum = bottomWheelErrorSum + bI*bottomWheelError*.02;

    topWheelErrorSum = turret.limit(topWheelErrorSum, .1, -.1);
    bottomWheelErrorSum = turret.limit(bottomWheelErrorSum, .1, -.1);
    
    rotationError = horizontalOffset-1.5;
    tiltError = verticalOffset + distanceToTarget/3.5;

    rotationPower = rotationError/30;
    tiltPower = -tiltError/4;

    double minError = .15;
    double topFeedForward, bottomFeedForward, feederFeedForward;

    topFeedForward = desiredTopWheelRPM/topShooterMax;
    bottomFeedForward = desiredBottomWheelRPM/bottomShooterMax;
    feederFeedForward = feederDesired/feederMax;

    isShooterUpToSpeed = (topWheelError <= minError && bottomWheelError <=minError);
    
    if(isShooterUpToSpeed)
    {
      turret.isShooterUpToSpeed = true;
    }
    else 
    {
      turret.isShooterUpToSpeed = false;
    }
    hasShotAtLeastOnce = turret.turretBallCount >0;
    SmartDashboard.putBoolean("has Shot at least once", hasShotAtLeastOnce);
    if(hasShotAtLeastOnce && turret.turretBallCount >= 3)
    {
      hasShotAtLeastOnce = false;
    }
    if(isTargetWithinRange)
    {
      count=1;
    }
    if(RobotContainer.activeBallCount==0)
    {
      count = 0; 
    }
    boolean isReadyShoot = RobotContainer.activeBallCount>=0 && !isDriving;
   
    topWheelInput = (isReadyShoot) ? (topFeedForward *tF) + (topWheelError *tP) + (topWheelErrorSum) : 0;
    bottomWheelInput = (isReadyShoot) ? (bottomFeedForward *bF) + (bottomWheelError *bP) + (bottomWheelErrorSum) : 0;
    feederInput = isShooterUpToSpeed ? (feederFeedForward *fF) + (feederError * fP) : 0;
    rotationInput = !hasShotAtLeastOnce &&  !isShooterUpToSpeed && !isTargetWithinRange && targetExist ? (rotationPower * rP) + (rotationErrorSum * rI): 0;
    tiltInput = !hasShotAtLeastOnce && !isShooterUpToSpeed && !isTargetWithinRange && targetExist ? tiltPower * tiltP: 0;
    storageInput = isShooterUpToSpeed  ? (storageError * storageP): 0;


    if(currentStoragePos >= -4)
    {
      storageInput = -.25;
    }


  //  turret.aimTurret(rotationInput, tiltInput);
    turret.runBallFeeder(-feederInput);
    turret.runShooterPower(topWheelInput, bottomWheelInput);
    storage.runConveyor(storageInput);   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    SmartDashboard.putString("Turret Command Running: ", "No Command Running");
    SmartDashboard.putString("Storage Command Running: ", "No Command Running");
    turret.stopAllTurretMotors();
    storage.runConveyor(0);   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
