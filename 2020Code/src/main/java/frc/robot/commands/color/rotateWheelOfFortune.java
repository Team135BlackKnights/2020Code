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
  public double manualSpinSpeed;

  public rotateWheelOfFortune(double manualSpin) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.colorWheel);
    manualSpinSpeed = manualSpin;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    isFinished=false;

    SmartDashboard.putString("Control Panel Command Running:", "rotate Wheel of Fortune");
    Robot.colorWheel.moveExtend(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    
    if (manualSpinSpeed != 0)//if = 0, then command is not manual control and skip this block
    {
      Robot.colorWheel.rotatorSpark.set(manualSpinSpeed);
      edu.wpi.first.wpilibj.Timer t = new edu.wpi.first.wpilibj.Timer();
      t.start();
      double starttime = t.get();
      while ((t.get() - starttime) < .25 )//runs motor for .25 seconds
      {}
      Robot.colorWheel.rotatorSpark.set(0);
      isFinished = true;
      return;
    }
    
    //Sets the desired color to the color given by the game
    String DesiredColor = Robot.colorWheel.gameColor();
    SmartDashboard.putString("FMS Readout", DesiredColor);
    //If the desired color isn't empty, rotate the wheel at 80% power until it is detected 
    if (DesiredColor != "No Color") {
      Robot.colorWheel.getToColor(DesiredColor, .25);
      if (Robot.colorWheel.atDesiredRoations){ //2If the wheel has been spun the desired amount, it is finished
        isFinished = true;
        Robot.colorWheel.stopControlPanel();
        }
    }
    else {
      //If the desired color is no color, rotate the wheel four times at 80% power
      Robot.colorWheel.rotateColorWheel(.8, 2.75);
      if (Robot.colorWheel.checkForColor() != Robot.colorWheel.desiredColor) {
        isFinished = true;
        Robot.colorWheel.stopControlPanel();
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
    Robot.colorWheel.stopControlPanel();
    Robot.colorWheel.rotatorSpark.set(0);
    SmartDashboard.putString("Control Panel Command Running:", "No command Running");
    Robot.colorWheel.atDesiredRoations=false;
    Robot.colorWheel.moveExtend(false);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}

