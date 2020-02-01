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
  /**
   * Creates a new TargetTurret.
   */
  private final Turret turret;
  private final TurretLimelight turretLimelight;
  private final ImprovedJoystick _joystick;
  public double horizontalOffset, verticalOffset, targetArea, anglularOffset;
  public boolean targetExist;


  public TargetTurret(Turret turretSubsystem, TurretLimelight limelightSubsystem, Joystick joystick) 
  {
    _joystick = new ImprovedJoystick(joystick);
    turret = turretSubsystem;
    turretLimelight = limelightSubsystem;
    addRequirements(turret);
    addRequirements(turretLimelight);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    SmartDashboard.putString("Turret Command Running: ", "targetTurret");
    turretLimelight.initLimelight(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetExist = turretLimelight.GetLimelightData()[0] >= 1 ? true : false;
    verticalOffset = turretLimelight.GetLimelightData()[2];
    horizontalOffset = turretLimelight.GetLimelightData()[1];
    anglularOffset = turretLimelight.GetLimelightData()[4];
    targetArea = turretLimelight.GetLimelightData()[3];
  
    double rotationPower, tiltPower;

    double rP = 1, tP = 1;

    rotationPower = horizontalOffset/8;
    tiltPower = verticalOffset/8;

    double minPower = .3;

    double rotationDirection = horizontalOffset > 0 ? -1: 1;
    boolean isPOVLeft, isPOVRight, isPOVUp, isPOVDown, isPOVTopRight, isPOVBottomRight, isPOVBottomLeft, isPOVTopLeft;


    isPOVUp = _joystick.isPovDirectionPressed(0);
    isPOVRight = _joystick.isPovDirectionPressed(1);
    isPOVDown = _joystick.isPovDirectionPressed(2);
    isPOVLeft = _joystick.isPovDirectionPressed(3);

    isPOVTopRight = _joystick.isPovDirectionPressed(4);
    isPOVBottomRight = _joystick.isPovDirectionPressed(5);
    isPOVBottomLeft = _joystick.isPovDirectionPressed(6);
    isPOVTopLeft = _joystick.isPovDirectionPressed(7);
    
    if(isPOVUp)
    {
      rotationPower = 0;
      tiltPower = .6;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } 
    else 
    if(isPOVRight)
    {
      rotationPower = .6;
      tiltPower = 0;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } 
    else 
    if(isPOVDown)
    {
      rotationPower = 0;
      tiltPower = -.6;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } 
    else
    if(isPOVLeft)
    {
      rotationPower = -.6;
      tiltPower = 0;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } 
    else 
    if(isPOVTopRight)
    {
      rotationPower = .6;
      tiltPower = .6;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } 
    else 
    if(isPOVBottomRight)
    {
      rotationPower = .6;
      tiltPower = -.6;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } 
    else 
    if(isPOVBottomLeft)
    {
      rotationPower = -.6;
      tiltPower = -.6;
      SmartDashboard.putString("Turret State:", "Driver Override");

    } 
    else 
    if(isPOVTopLeft)
    {
      rotationPower = -.6;
      tiltPower = .6;
      SmartDashboard.putString("Turret State:", "Driver Override");

    }
    else 
    if(targetExist)
    {
      rotationPower = (rotationPower * rP) + (minPower* rotationDirection);
      tiltPower = (tiltPower * tP) + (minPower);
      SmartDashboard.putString("Turret State:", "Auto Targetting");
    }
    else 
    {
      rotationPower = 0;
      tiltPower = 0;
      SmartDashboard.putString("Turret State:", "No Target");

    }
  turret.aimTurret(rotationPower, tiltPower);

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
