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
  
  public runTurretAuton(Turret _turret, TurretLimelight _limelight, Storage _storage) 
  {
    turret = _turret;
    limelight = _limelight;
    storage = _storage;
    addRequirements(turret, limelight, storage);//testp
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    SmartDashboard.putString("Turret Command Running: ", "runTurretAuton");
    SmartDashboard.putString("Storage Command Running: ", "runTurretAuton");
    limelight.initLimelight(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
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
    verticalOffset = limelight.GetLimelightData()[2] +distanceToTarget/1.5;
    horizontalOffset = limelight.GetLimelightData()[1] -distanceToTarget/5.25;

    currentTopWheelRPM = turret.getTopWheelRPM();
    currentBottomWheelRPM = turret.getBottomWheelRPM();
    feederActual = turret.getFeederRPM();
    storageActual = storage.getConveyorVel();
    currentStoragePos = storage.getConveyorRotations();
    MovingAverage smoothStorage = new MovingAverage(25);
    double averagedStorage = smoothStorage.process((float)storageActual);
    
    isDriving = Math.abs(RobotContainer.drive.getLinearMps()) >.5;
    isTargetWithinRange = (Math.abs(verticalOffset) < 1.5 && Math.abs(horizontalOffset) < 1.5);

    feederMax = 5250;
    topShooterMax = 5600;
    bottomShooterMax = 5200;
    storageMax = 5200;

    desiredTopWheelRPM = 2000;
    desiredBottomWheelRPM = desiredTopWheelRPM * 1.25;
    feederDesired = -.35*feederMax;
    storageDesired = -1800;
    
    topWheelError = desiredTopWheelRPM - currentTopWheelRPM;
    bottomWheelError = desiredBottomWheelRPM - currentBottomWheelRPM;
    feederError = feederDesired - feederActual;
    storageError = storageDesired-averagedStorage;
    storageError = storageError/storageMax;

    topWheelInput = (desiredTopWheelRPM + topWheelError)/topShooterMax;
    bottomWheelInput = (desiredBottomWheelRPM + bottomWheelError)/bottomShooterMax;
    feederInput = (desiredBottomWheelRPM + feederError)/feederMax;
    

    double tP, bP, fP, tI, bI, rP, rI, tiltP, storageP, 
    rotationErrorSum, topWheelErrorSum, bottomWheelErrorSum;

    rotationErrorSum =+ horizontalOffset*.02;
    topWheelErrorSum =+ topWheelInput*.02;
    bottomWheelErrorSum =+ bottomWheelInput*.02;
    
    rotationError = horizontalOffset;
    tiltError = verticalOffset + distanceToTarget/3.5;

    rotationPower = rotationError/60;
    tiltPower = -tiltError/4;

    double minError = 100;
    
    isShooterUpToSpeed = (topWheelError <= minError && bottomWheelError <=minError);

    if(isShooterUpToSpeed)
    {
      turret.isShooterUpToSpeed = true;
    }
    else 
    {
      turret.isShooterUpToSpeed = false;
    }


    rP = 1.4; rI = .25; tiltP = .87; storageP = 2.1;
    bI = .4; tI = .4; tP = .99; bP = .968; fP = .862;
    int count = 0; 
    if(isTargetWithinRange)
    {
      count++;
    }
    boolean isReadyShoot = count>=1 && RobotContainer.activeBallCount>=0 && !isDriving;
   
    topWheelInput = (isReadyShoot) ? (topWheelInput * tP) + (topWheelErrorSum + tI) : 0;
    bottomWheelInput = (isReadyShoot) ? (bottomWheelInput * bP) + (bottomWheelErrorSum + bI) : 0;
    feederInput = isShooterUpToSpeed ? feederInput*fP : 0;
    rotationInput = !isShooterUpToSpeed && !isTargetWithinRange && targetExist ? (rotationPower * rP) + (rotationErrorSum * rI): 0;
    tiltInput = !isShooterUpToSpeed && !isTargetWithinRange && targetExist ? tiltPower * tiltP: 0;
    storageInput = isShooterUpToSpeed  ? (storageError * storageP): 0;


    if(currentStoragePos >= -2.5)
    {
      storageInput = -.25;
    }


    turret.aimTurret(rotationInput, tiltInput);
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
    return false;
  }
}
