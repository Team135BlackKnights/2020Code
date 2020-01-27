/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.endgame;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class runWinch extends Command {

  public double _power;

  public runWinch(double power){

    this._power = power;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeat  p/ eg.ublicn this/Command is scheduled to run
  @Override
  protected void execute() {
    Robot.endgame.runWinchSpark(_power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once aowe requr, CAinished returns true
  @Override
  protected void end() 
  {
    Robot.endgame.runWinchSpark(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
