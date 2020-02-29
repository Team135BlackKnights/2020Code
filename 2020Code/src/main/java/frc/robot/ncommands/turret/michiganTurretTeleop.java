/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.Storage;
import frc.robot.nsubsystems.Turret;
import frc.robot.nsubsystems.TurretLimelight;
import frc.robot.util.ImprovedJoystick;
import frc.robot.util.MovingAverage;

public class michiganTurretTeleop extends CommandBase {
  /**
   * Creates a new michiganTurretTeleop.
   */
  private Turret turret;
  private Storage storage;
  private TurretLimelight limelight;
  private ImprovedJoystick joystick;
  public double topErrorSum, bottomErrorSum;
  public boolean readyToShoot, isShooting, isDriving, targetTurret, overrideTurret;  
  private long furtherTime = 0;

  MovingAverage averagedStorageRPM = new MovingAverage(25);
  
  public michiganTurretTeleop(Turret _turret, TurretLimelight _limelight, Storage _storage, Joystick _joystick) 
  {
    turret = _turret;
    limelight = _limelight;
    storage = _storage;
    joystick = new ImprovedJoystick(_joystick);
    addRequirements(turret, limelight, storage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    SmartDashboard.putString("Turret Command Running: ", "Michigan Turret Teleop");
    SmartDashboard.putString("Storage Command Running: ", "Michigan Turret Teleop");
  
    topErrorSum = 0;
    bottomErrorSum = 0;
    limelight.initLimelight(0, 0);
    isShooting = false;
    overrideTurret = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    boolean targetExist;
    
    isDriving = Math.abs(RobotContainer.drive.getLinearMps()) > .25;

    double verticalOffset, horizontalOffset, distanceToTarget,
          desiredTopRPM, desiredBottomRPM, desiredFeederRPM, desiredStorageRPM,
          topMaxRPM, bottomMaxRPM, feederMaxRPM, storageMaxRPM, 
          topCurrentRPM, bottomCurrentRPM, storageCurrentRPM, 
          adjustedStorageRPM, adjustedVerticalOffset, adjustedHorizontalOffset, rotationErrorSum, rotationIntegral,
          topFeedForward, bottomFeedForward, feederFeedForward,
          topInput, bottomInput, feederInput, storageInput, rotationInput, elevationInput,
          minShooterError, rotationError, elevationError, topError, bottomError, storageError,
          elevationOverride, rotationOverride, currentStoragePosition, 
          
          rP,eP,tP,bP,sP,
          rI, tI, bI,
          tF, bF, fF;

    boolean isPOVLeft, isPOVRight, isPOVUp, isPOVDown, isPOVTopRight, isPOVBottomRight, isPOVBottomLeft, isPOVTopLeft;

          currentStoragePosition = storage.getConveyorRotations();

          targetExist = limelight.GetLimelightData()[0] >=1 ;
          verticalOffset = limelight.GetLimelightData()[2];
          horizontalOffset = limelight.GetLimelightData()[1];
          distanceToTarget = limelight.distToTarget();

          topMaxRPM = 5100; bottomMaxRPM = 5000; feederMaxRPM = 5000; storageMaxRPM = 5200;
          desiredTopRPM = -18.526*Math.pow(distanceToTarget,2) + 258.5128*distanceToTarget + 1667.3675;
          desiredBottomRPM = desiredTopRPM*1.25;

          if(joystick.getJoystickButtonValue(2))
          {
            desiredTopRPM = 1500;
          }

          desiredFeederRPM = .35*feederMaxRPM;
          desiredStorageRPM = -1550;

          adjustedVerticalOffset = verticalOffset +distanceToTarget/3.5;
          adjustedHorizontalOffset = horizontalOffset - distanceToTarget/4.4 -1.5;

          rotationError = (adjustedHorizontalOffset/30);
          elevationError = (adjustedVerticalOffset/4);

          rotationErrorSum =+ adjustedHorizontalOffset;
          rotationIntegral = rotationErrorSum*.02;

          rP = 1.4; eP = .87; rI = .25;
          rotationInput = (rotationError *rP) + (rotationIntegral* rI);
          elevationInput = (elevationError * eP);

          topCurrentRPM = turret.getTopWheelRPM();
          bottomCurrentRPM = turret.getBottomWheelRPM();
          storageCurrentRPM = storage.getConveyorVel();

          adjustedStorageRPM = averagedStorageRPM.process((float)storageCurrentRPM);

          storageError = (desiredStorageRPM-adjustedStorageRPM)/storageMaxRPM;

          sP = 2.1;
          storageInput = storageError *sP;

          topError =(desiredTopRPM - topCurrentRPM)/topMaxRPM;
          bottomError = (desiredBottomRPM - bottomCurrentRPM)/bottomMaxRPM;

          tF = .85; bF = .85; tP = .5; bP = .94; tI = 2.5; bI = 2; fF = .892;
          
          topErrorSum = topErrorSum + (tI*topError *.02);
          bottomErrorSum =bottomErrorSum + (bI*bottomError *.02);

          topErrorSum = turret.limit(topErrorSum, .1, -.1);
          bottomErrorSum = turret.limit(bottomErrorSum, .1, -.1);

          topFeedForward = (desiredTopRPM)/topMaxRPM;
          bottomFeedForward = desiredBottomRPM/bottomMaxRPM;
          feederFeedForward = desiredFeederRPM/feederMaxRPM;

          topInput = (topFeedForward * tF) + (topError*tP) + (topErrorSum);
          bottomInput = (bottomFeedForward * bF ) + (bottomError*bP) + (bottomErrorSum);
          feederInput = (feederFeedForward * fF);

          minShooterError = .15;

          readyToShoot = (topError <=minShooterError && bottomError <= minShooterError);

          if(joystick.getJoystickButtonValue(1) && !isDriving)
          {
            turret.runShooterPower(topInput, bottomInput);
            isShooting = true;
            if(readyToShoot)
            {
              turret.runBallFeeder(feederInput);
              storage.runConveyor(storageInput);
            }
          }
          else 
          {
            isShooting = false;
          }

          long timeNow = System.currentTimeMillis();

          if (joystick.getJoystickButtonValue(6) && timeNow >= furtherTime) {
            furtherTime = timeNow + 150;
            overrideTurret = !overrideTurret;
          }

          isPOVUp = joystick.isPovDirectionPressed(0);
          isPOVRight = joystick.isPovDirectionPressed(1);
          isPOVDown = joystick.isPovDirectionPressed(2);
          isPOVLeft = joystick.isPovDirectionPressed(3);
      
          isPOVTopRight = joystick.isPovDirectionPressed(4);
          isPOVBottomRight = joystick.isPovDirectionPressed(5);
          isPOVBottomLeft = joystick.isPovDirectionPressed(6);
          isPOVTopLeft = joystick.isPovDirectionPressed(7);
          
          rotationOverride = .75;
          elevationOverride =.6;
          
          if(isShooting)
          {
            targetTurret = false;
          }
          else if(overrideTurret)
          {
            targetTurret = false;
          }
          else
          {
            targetTurret = true;
          }

          if(targetTurret)
          {
            limelight.initLimelight(0, 0);
          }
          else
          {
            limelight.initLimelight(1, 1);
          }
          
          if (isPOVUp) {
          
            turret.aimTurret(0, elevationInput);
          } else if (isPOVRight) {
        
            turret.aimTurret(rotationOverride, 0);      
          } else if (isPOVDown) {
            turret.aimTurret(0, -elevationInput);
          } else if (isPOVLeft) {
            turret.aimTurret(-rotationOverride, 0);      
          } else if (isPOVTopRight) {
            turret.aimTurret(rotationOverride, elevationInput);
          } else if (isPOVBottomRight) {
            turret.aimTurret(rotationOverride, -elevationOverride);
          } else if (isPOVBottomLeft) {
            turret.aimTurret(-rotationOverride, -elevationInput);
          } else if (isPOVTopLeft) {
            turret.aimTurret(-rotationOverride, elevationInput);
          } else if (targetExist && targetTurret) {
            turret.aimTurret(rotationInput, elevationInput);
          } else {
            turret.aimTurret(0, 0);
          }
          boolean isButton7, isButton8;
          isButton7 = joystick.getJoystickButtonValue(7);
          isButton8 = joystick.getJoystickButtonValue(8);

          if(currentStoragePosition >= -4  && !readyToShoot && !(isButton7 || isButton8))
          {
            storage.runConveyor(-.25);
            RobotContainer.intake.runRoller(0);
           RobotContainer.turret.runBallFeeder(0);
          }
          else if(isButton7)
          {
            storage.runConveyor(.85);
            storage.resetConveyorEncoder();
            RobotContainer.intake.runRoller(-.3);
            turret.runBallFeeder(.2);
            if(RobotContainer.intake.isRollerLowered())
            {
            RobotContainer.intake.raiseLower(false);
            }
          }
          else if(isButton8)
          {
            RobotContainer.intake.runRoller(.3);
            storage.resetConveyorEncoder();
            storage.runConveyor(-.25);
          }
          else 
          {
            storage.runConveyor(0);
          }


        }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    turret.stopAllTurretMotors();
    storage.runConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
