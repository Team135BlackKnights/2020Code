 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.color;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class rotateWheelOfFortune extends Command {

  public boolean isFinished;


  public rotateWheelOfFortune() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.colorWheel);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("Control Panel Command Running:", "rotate Wheel of Fortune");

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //Sets the desired color to the color given by the game
    String DesiredColor = Robot.colorWheel.gameColor();

    if(DesiredColor != "No Color")
    {
      Robot.colorWheel.getToSpecifiedColor(DesiredColor);
      isFinished = false; 
    }
    else if(DesiredColor == "No Color" && !Robot.colorWheel.atDesiredRoations)
    {
      Robot.colorWheel.rotateXRotations(4);
      isFinished = false;
    }
    else 
    {
      Robot.colorWheel.stopControlPanel();
      isFinished = true;
    }  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.colorWheel.stopControlPanel();
    SmartDashboard.putString("Control Panel Command Running:", "No command Running");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}

