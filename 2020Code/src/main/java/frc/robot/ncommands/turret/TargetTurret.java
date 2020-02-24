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
import frc.robot.nsubsystems.*;
import frc.robot.util.ImprovedJoystick;

public class TargetTurret extends CommandBase {

  double tiltPowerPreset = .6;
  double rotationPowerPreset = .75;
  private final Turret turret;
  private final TurretLimelight turretLimelight;
  private final ImprovedJoystick _joystick;
  private double horizontalOffset, verticalOffset;
  private boolean targetExist;
  private boolean targetTurret;
  private boolean overrideTurret; 

  private long furtherTime = 0;
  private double previousError;

  public TargetTurret(Turret turretSubsystem, TurretLimelight limelightSubsystem, Joystick joystick) {
    _joystick = new ImprovedJoystick(joystick);
    turret = turretSubsystem;
    turretLimelight = limelightSubsystem;
    addRequirements(turret);
    addRequirements(turretLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetTurret = false;
    overrideTurret = false; 
    previousError = 0; 
    SmartDashboard.putString("Turret Command Running: ", "targetTurret");
    turretLimelight.initLimelight(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetExist = turretLimelight.GetLimelightData()[0] >= 1 ? true : false;
    verticalOffset = turretLimelight.GetLimelightData()[2];
    horizontalOffset = turretLimelight.GetLimelightData()[1] +.5;
    
    boolean isShooting = turret.getTopWheelRPM() > 1250;
    double distToTarget = turretLimelight.distToTarget();

    verticalOffset = verticalOffset + distToTarget/3.5;
    double rotationPower, tiltPower;
    double rotationHelper = distToTarget/6;
    SmartDashboard.putNumber("ROtatoinhelper", rotationHelper);
    double rP = 1.4, tP = .87, rI = .25, rD = .00;
    rotationPower = horizontalOffset /30;
    tiltPower = -verticalOffset / 4;

    //rotationPower = rotationPower+rotationHelper;

    double derivative = (rotationPower- previousError)/.02;

    double minPower = .01;

    double rotationDirection = horizontalOffset > 0 ? -1 : 1;
    boolean isPOVLeft, isPOVRight, isPOVUp, isPOVDown, isPOVTopRight, isPOVBottomRight, isPOVBottomLeft, isPOVTopLeft;

    double totalError =+ horizontalOffset;

    double rIntegral = totalError*.02;

    isPOVUp = _joystick.isPovDirectionPressed(0);
    isPOVRight = _joystick.isPovDirectionPressed(1);
    isPOVDown = _joystick.isPovDirectionPressed(2);
    isPOVLeft = _joystick.isPovDirectionPressed(3);

    isPOVTopRight = _joystick.isPovDirectionPressed(4);
    isPOVBottomRight = _joystick.isPovDirectionPressed(5);
    isPOVBottomLeft = _joystick.isPovDirectionPressed(6);
    isPOVTopLeft = _joystick.isPovDirectionPressed(7);

    long timeNow = System.currentTimeMillis();
    
    
    
    
    if (_joystick.getJoystickButtonValue(6) && timeNow >= furtherTime) {
      furtherTime = timeNow + 100;
      overrideTurret = !overrideTurret;
    }

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
    
    SmartDashboard.putBoolean("Auto Targeting turret: ", targetTurret);

    if (isPOVUp) {
      rotationPower = 0;
      tiltPower = tiltPowerPreset;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } else if (isPOVRight) {
      rotationPower = rotationPowerPreset;
      tiltPower = 0;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } else if (isPOVDown) {
      rotationPower = 0;
      tiltPower = -tiltPowerPreset;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } else if (isPOVLeft) {
      rotationPower = -rotationPowerPreset;
      tiltPower = 0;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } else if (isPOVTopRight) {
      rotationPower = rotationPowerPreset;
      tiltPower = tiltPowerPreset;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } else if (isPOVBottomRight) {
      rotationPower = rotationPowerPreset;
      tiltPower = -tiltPowerPreset;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } else if (isPOVBottomLeft) {
      rotationPower = -rotationPowerPreset;
      tiltPower = -tiltPowerPreset;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } else if (isPOVTopLeft) {
      rotationPower = -rotationPowerPreset;
      tiltPower = tiltPowerPreset;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } else if (targetExist && targetTurret) {
      rotationPower = (rotationPower * rP) + (rIntegral * rI) + (derivative *rD) + (minPower * rotationDirection);
      tiltPower = (tiltPower * tP);
      SmartDashboard.putString("Turret State:", "Auto Targetting");
    } else {
      rotationPower = 0;
      tiltPower = 0;
      SmartDashboard.putString("Turret State:", "No Target");
    }

    SmartDashboard.putNumber("INtegral", rIntegral * rI);
    SmartDashboard.putNumber("Target Turret Rotation Power:", rotationPower);
    SmartDashboard.putNumber("Target Turret Tilt Power:", tiltPower);

    turret.aimTurret(rotationPower, tiltPower);
    previousError = horizontalOffset / 40;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
