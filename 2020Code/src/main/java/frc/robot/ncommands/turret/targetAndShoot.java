
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.Storage;
import frc.robot.nsubsystems.Turret;
import frc.robot.util.ImprovedJoystick;
import frc.robot.util.MotorControl;

public class targetAndShoot extends CommandBase {

  Turret turret;
  boolean isFinished, targetTurret, overrideTurret;
  double errorSum;
  int ballsToShoot;
  private long furtherTime = 0;
  ImprovedJoystick joystick;
  Storage storage;

  public targetAndShoot(Turret _turret, Storage _storage, Joystick _joystick, int autonBalls) {
    turret = _turret;
    joystick = new ImprovedJoystick(_joystick);
    ballsToShoot = autonBalls;
    storage = _storage;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.initLimelight(0, 0);

    SmartDashboard.putString("Turret Command Running: ", "Target and shoot Turret");
    targetTurret = false;
    overrideTurret = false;
    errorSum = 0;
    turret.ballsShot = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isTargetValid, isDriving, isAuton, isShooting, isTargetWithinRange;
    boolean isPOVLeft, isPOVRight, isPOVUp, isPOVDown, isPOVTopRight, isPOVBottomRight, isPOVBottomLeft, isPOVTopLeft;

    long timeNow = System.currentTimeMillis();

    isAuton = false;
    
    double distanceToTarget, horizontalOffset, rotationError, hoodDesired, hoodActual, hoodError, desiredRPM,
        overrideRPM, actualRPM, rpmError, maxRPM, minRotationError, minHoodError, feedForwardRPM, shooterInput,
        rotationInput, hoodInput;

    distanceToTarget = turret.getAreaDistance();
    SmartDashboard.putNumber("distance to Target", distanceToTarget);
    horizontalOffset = turret.limelightData[1] - 3.5;
    isTargetValid = turret.limelightData[0] >= 1;
    isDriving = RobotContainer.drive.getLinearMps() >= .15;

    minRotationError = .025;
    minHoodError = 2;

    if (joystick.getJoystickButtonValue(6) && timeNow >= furtherTime) {
      furtherTime = timeNow + 100;
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

    maxRPM = 6000;

    rotationError = horizontalOffset / 30 ;

    hoodDesired = (-1.1855* Math.pow(distanceToTarget, 2)) + (22.115 * distanceToTarget) + 78.0045;
    SmartDashboard.putNumber("Desired Hood Pos", hoodDesired);
    hoodActual = turret.getHoodPos();
    hoodError = hoodDesired - hoodActual;

    overrideRPM = 5200 * joystick.getJoystickSlider();
    SmartDashboard.putNumber("Override RPM", overrideRPM);
    actualRPM = turret.getShooterVel();

    double rP, hP, sF, sP, sI;

    rP = 1;
    hP = .1;
    sF = .625;
    sP = 2;
    sI = 0;

    

    isShooting =  (actualRPM > 2000);

    if (isShooting) {
      targetTurret = false;
    } else if (overrideTurret) {
      targetTurret = false;
    } else {
      targetTurret = true;
    }

    if (isAuton) {
      desiredRPM = 2500;
    } else if (!isAuton && overrideRPM == 5200 || overrideRPM == 0 && targetTurret) {
      desiredRPM = (20.9988 * Math.pow(distanceToTarget, 2))  + (122.8191* distanceToTarget) + 2285.4107;
    } else {
      desiredRPM = overrideRPM;
    }


    SmartDashboard.putNumber("shooter desired RPM", desiredRPM);

    rpmError = (desiredRPM - actualRPM)/maxRPM;
    feedForwardRPM = desiredRPM / maxRPM;

    errorSum = errorSum + (rpmError * sI * .02);


    turret.isReadyForBall = (!(desiredRPM == 0) && Math.abs(rpmError) <= .10);


    if(overrideTurret)
    {
      turret.initLimelight(1,0);
    }
    else 
    {
      turret.initLimelight(0,0);
    }


    rotationInput = rotationError * rP;
    hoodInput = hoodError * hP;
    
    shooterInput = (feedForwardRPM * sF) + (rpmError * sP) + (errorSum);
    shooterInput = MotorControl.limit(shooterInput, 1, 0);
    SmartDashboard.putNumber("shooterInput", shooterInput);
    isTargetWithinRange = ((isTargetValid && Math.abs(rotationError) < minRotationError
       && Math.abs(hoodError) < minHoodError) || isShooting);


        SmartDashboard.putBoolean("is Target Within range ", isTargetWithinRange);

    if (isAuton && ballsToShoot > 3 && !isDriving) {
      if (turret.ballsShot < 3 && isTargetWithinRange) {
        turret.runShooter(shooterInput);
      } else if (turret.ballsShot < ballsToShoot && Math.abs(RobotContainer.drive.getLeftMetres()) > 1
          && isTargetWithinRange) {
        turret.runShooter(shooterInput);
      } else {
        turret.runShooter(0);
      }
    } else if (isAuton && ballsToShoot == 3 && !isDriving) {
      if (turret.ballsShot < 3 && isTargetWithinRange) {
        turret.runShooter(shooterInput);
      } else {
        turret.runShooter(0);
      }
    } else if ( joystick.getJoystickButtonValue(1) && isTargetWithinRange && !isDriving) {
      turret.runShooter(shooterInput);
    } else {
      turret.runShooter(0);
    }

    if (isAuton) {
      if (!isTargetWithinRange && targetTurret) {
        turret.aimTurret(rotationInput, hoodInput);
      } else if (!isTargetWithinRange && !targetTurret) {
        turret.aimTurret(0, 0);
      }
    } else if (isPOVUp) {
      turret.aimTurret(0, .45);
      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVRight) {
      turret.aimTurret(.45, 0);
      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVDown) {
      turret.aimTurret(0, -.45);
      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVLeft) {
      turret.aimTurret(-.45, 0);

      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVTopRight) {
      turret.aimTurret(.45, .45);

      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVBottomRight) {
      turret.aimTurret(.45, -.45);
      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVBottomLeft) {
      turret.aimTurret(-.45, -.45);

      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVTopLeft) {
      turret.aimTurret(-.45, .45);

      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (!isTargetWithinRange && isTargetValid && targetTurret) {
      turret.aimTurret(rotationInput, hoodInput);

      SmartDashboard.putString("Turret State: ", "Auto Targetting");
    } else {
      turret.aimTurret(0, 0);

      SmartDashboard.putString("Turret State: ", "No Target");
    }

    if(turret.isReadyForBall)
    {
      turret.runIndexer(.4);
      // RobotContainer.storage.runConveyor(-1);
      //storage.runConveyor(1);
    }
    else 
    {
      turret.runIndexer(0);
      RobotContainer.storage.runConveyor(0);
    }
    SmartDashboard.putBoolean("is ready for ball", turret.isReadyForBall);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopAllMotors();
    turret.initLimelight(1, 1);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}