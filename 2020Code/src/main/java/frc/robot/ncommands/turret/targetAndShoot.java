/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.Turret;
import frc.robot.util.ImprovedJoystick;

public class targetAndShoot extends CommandBase {

  Turret turret;
  boolean isFinished, targetTurret, overrideTurret;
  double errorSum;
  int ballsToShoot;
  private long furtherTime = 0;
  ImprovedJoystick joystick;

  public targetAndShoot(Turret _turret, Joystick _joystick, int autonBalls) {
    turret = _turret;
    joystick = new ImprovedJoystick(_joystick);
    ballsToShoot = autonBalls;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.initLimelight(0, 0);

    SmartDashboard.putString("New Turret Command Running: ", "set Turret To Pos");
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

    isAuton = Timer.getMatchTime() <= 15;
    double distanceToTarget, horizontalOffset, rotationError, hoodDesired, hoodActual, hoodError, desiredRPM,
        overrideRPM, actualRPM, rpmError, maxRPM, minRotationError, minHoodError, feedForwardRPM, shooterInput,
        rotationInput, hoodInput;

    distanceToTarget = turret.distanceToTarget();
    horizontalOffset = turret.limelightData[1];
    isTargetValid = turret.limelightData[0] >= 1;
    isDriving = RobotContainer.drive.getLinearMps() >= .15;

    minRotationError = 1;
    minHoodError = 5;

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

    maxRPM = 5100;

    rotationError = horizontalOffset / 30;

    hoodDesired = 1 / distanceToTarget * 120; // TODO determine a relationship between the two with an equation
    hoodActual = turret.getHoodPos();
    hoodError = hoodDesired - hoodActual;

    overrideRPM = maxRPM * joystick.getJoystickThrottle();
    actualRPM = turret.getShooterVel();

    double rP, hP, sF, sP, sI;

    rP = 1;
    hP = 1;
    sF = 1;
    sP = 0;
    sI = 0;

    if (isAuton) {
      desiredRPM = 4600;
    } else if (!isAuton && overrideRPM == 5100 || overrideRPM == 0) {
      desiredRPM = 4600;
    } else {
      desiredRPM = overrideRPM;
    }

    rpmError = desiredRPM - actualRPM;
    feedForwardRPM = desiredRPM / maxRPM;

    errorSum = errorSum + (rpmError * sI * .02);

    isShooting = (!(desiredRPM == 0) && actualRPM > 2000);
    turret.isReadyForBall = (!(desiredRPM == 0) && Math.abs(rpmError) <= 150);

    if (isShooting) {
      targetTurret = false;
    } else if (overrideTurret) {
      targetTurret = false;
    } else {
      targetTurret = true;
    }

    rotationInput = rotationError * rP;
    hoodInput = hoodError * hP;

    shooterInput = feedForwardRPM * sF + rpmError * sP + errorSum;

    isTargetWithinRange = ((isTargetValid && Math.abs(rotationError) < minRotationError
        && Math.abs(hoodError) < minHoodError) || isShooting);

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
    } else if (isTargetWithinRange && joystick.getJoystickButtonValue(1) && !isDriving) {
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
      turret.aimTurret(0, .65);
      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVRight) {
      turret.aimTurret(.65, 0);
      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVDown) {
      turret.aimTurret(0, -.65);
      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVLeft) {
      turret.aimTurret(-.65, 0);

      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVTopRight) {
      turret.aimTurret(.65, .65);

      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVBottomRight) {
      turret.aimTurret(.65, -.65);
      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVBottomLeft) {
      turret.aimTurret(-.65, -.65);

      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (isPOVTopLeft) {
      turret.aimTurret(-.65, .65);

      SmartDashboard.putString("Turret State: ", "Driver Override");

    } else if (!isTargetWithinRange && isTargetValid && targetTurret) {
      turret.aimTurret(rotationInput, hoodInput);

      SmartDashboard.putString("Turret State: ", "Auto Targetting");
    } else {
      turret.aimTurret(0, 0);

      SmartDashboard.putString("Turret State: ", "No Target");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}