 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.color;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ColorWheel;

public class rotateWheelOfFortune extends Command {

  public boolean isFinished;


  public rotateWheelOfFortune() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.colorWheel);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //Sets the desired color to the color given by the game
    String DesiredColor = Robot.colorWheel.gameColor();

    //If the desired color isn't empty, rotate the wheel at 80% power until it is detected 
    if (DesiredColor != "No Color") {
      Robot.colorWheel.getToColor(DesiredColor, .8);
      if (Robot.colorWheel.atDesiredRoations){ // If the wheel has been spun the desired amount, it is finished
        isFinished = true;
      }
    }
    else { //If the desired color is no color, rotate the wheel four times at 80% power
      Robot.colorWheel.rotateColorWheel(.8, 4);
      if (Robot.colorWheel.checkForColor() != Robot.colorWheel.desiredColor) {
        isFinished = true;
      }
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
