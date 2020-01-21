/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class targetTurret extends Command {

  public double horizontalOffset, verticalOffset, targetArea, anglularOffset;
  public boolean targetExist;
  public targetTurret() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.turret);
    requires(Robot.turretlimelight);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("Turret Command Running: ", "targetTurret");
    Robot.turretlimelight.initLimelight(0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    targetExist = Robot.turretlimelight.GetLimelightData()[0] >= 1 ? true : false;
    verticalOffset = Robot.turretlimelight.GetLimelightData()[2];
    horizontalOffset = Robot.turretlimelight.GetLimelightData()[1];
    anglularOffset = Robot.turretlimelight.GetLimelightData()[4];
    targetArea = Robot.turretlimelight.GetLimelightData()[3];
  
  
    double rotationPower, tiltPower;

    double rP = 1, tP = 1;

    rotationPower = horizontalOffset/8;
    tiltPower = verticalOffset/8;

    double minPower = .3;

    double rotationDirection = horizontalOffset > 0 ? -1: 1;

    boolean isPOVLeft, isPOVRight, isPOVUp, isPOVDown, isPOVTopRight, isPOVBottomRight, isPOVBottomLeft, isPOVTopLeft;

    isPOVUp = Robot.oi.isPovDirectionPressed(Robot.oi.MANIP_JOYSTICK, Robot.oi.TOP_POV);
    isPOVRight = Robot.oi.isPovDirectionPressed(Robot.oi.MANIP_JOYSTICK, Robot.oi.RIGHT_POV);
    isPOVDown = Robot.oi.isPovDirectionPressed(Robot.oi.MANIP_JOYSTICK, Robot.oi.BOTTOM_POV);
    isPOVLeft = Robot.oi.isPovDirectionPressed(Robot.oi.MANIP_JOYSTICK, Robot.oi.LEFT_POV);

    isPOVTopRight = Robot.oi.isPovDirectionPressed(Robot.oi.MANIP_JOYSTICK, Robot.oi.TOP_RIGHT_POV);
    isPOVBottomRight = Robot.oi.isPovDirectionPressed(Robot.oi.MANIP_JOYSTICK, Robot.oi.BOTTOM_RIGHT_POV);
    isPOVBottomLeft = Robot.oi.isPovDirectionPressed(Robot.oi.MANIP_JOYSTICK, Robot.oi.BOTTOM_LEFT_POV);
    isPOVTopLeft = Robot.oi.isPovDirectionPressed(Robot.oi.MANIP_JOYSTICK, Robot.oi.TOP_LEFT_POV);
    

    if(isPOVUp)
    {
      rotationPower = 0;
      tiltPower = .6;
    } else 
    if(isPOVRight)
    {
      rotationPower = .6;
      tiltPower = 0;
    } else 
    if(isPOVDown)
    {
      rotationPower = 0;
      tiltPower = -.6;
    } else
    if(isPOVLeft)
    {
      rotationPower = -.6;
      tiltPower = 0;
    } else 
    if(isPOVTopRight)
    {
      rotationPower = .6;
      tiltPower = .6;
    } else 
    if(isPOVBottomRight)
    {
      rotationPower = .6;
      tiltPower = -.6;
    } else 
    if(isPOVBottomLeft)
    {
      rotationPower = -.6;
      tiltPower = -.6;
    } else 
    if(isPOVTopLeft)
    {
      rotationPower = -.6;
      tiltPower = .6;
    }
    else 
    if(targetExist)
    {
      rotationPower = (rotationPower * rP) + (minPower* rotationDirection);
      tiltPower = (tiltPower * tP) + (minPower);
    }
    else 
    {
      rotationPower = 0;
      tiltPower = 0;
    }




  Robot.turret.aimTurret(rotationPower, tiltPower);
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("Command Finished: ", "targetTurret");
    Robot.turret.stopTurret();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
