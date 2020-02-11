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
  double rotationPowerPreset = .25;
  private final Turret turret;
  private final TurretLimelight turretLimelight;
  private final ImprovedJoystick _joystick;
  private double horizontalOffset, verticalOffset;
  private boolean targetExist;
  private double loopRuns;
  private boolean targetTurret;

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
    loopRuns = 1;
    targetTurret = true;
    previousError = 0; 
    SmartDashboard.putString("Turret Command Running: ", "targetTurret");
    turretLimelight.initLimelight(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetExist = turretLimelight.GetLimelightData()[0] >= 1 ? true : false;
    verticalOffset = turretLimelight.GetLimelightData()[2];
    horizontalOffset = turretLimelight.GetLimelightData()[1];

    double distToTarget = turretLimelight.distToTarget();

    verticalOffset = verticalOffset + distToTarget/2;
    double rotationPower, tiltPower;

    double rP = .17, tP = 1, rI = .352, rD = 0;
    rotationPower = horizontalOffset / 70;
    tiltPower = -verticalOffset / 8;

    double derivative = (rotationPower- previousError)/.02;

    double minPower = .00;

    double rotationDirection = horizontalOffset > 0 ? 1 : -1;
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
      furtherTime = timeNow + 50;
      targetTurret = !targetTurret;
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
      rotationPower = (rotationPower * rP) + (rIntegral * rI) + (derivative *rD);
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
    previousError = horizontalOffset / 70;
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
