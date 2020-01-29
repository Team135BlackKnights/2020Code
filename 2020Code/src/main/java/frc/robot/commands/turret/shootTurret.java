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

public class shootTurret extends Command {
  public double _topShooterRPM, _bottomShooterRPM;
  public boolean isFinished;
  public shootTurret(double topShooterRPM, double bottomShooterRPM) {

  
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this._topShooterRPM = topShooterRPM;
    this._bottomShooterRPM = bottomShooterRPM;
    isFinished = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("Turret Command Running:","Shoot Turret");

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  
    //Robot.turret.topShooterSpark.set(-.75);
    //Robot.turret.bottomShooterSpark.set(.50);
    
    
    if(Robot.turret.isBallInShooter())
    {
      Robot.turret.runShooterRPM(_topShooterRPM, _bottomShooterRPM);
      Robot.turret.runBallFeeder(0);
      isFinished = false;
    }
    else if(!Robot.turret.isBallInShooter() && Robot.turret.turretBallCount == 0)
    {
      Robot.turret.runBallFeeder(.75);
      isFinished = false;
    }
    else if(Robot.turret.isBallInShooter() && Robot.turret.turretBallCount >= 1)
    {
      Robot.turret.turretBallCount--;
      isFinished = true;
    }
    else if(!Robot.turret.isBallInShooter() && Robot.turret.turretBallCount >=1)
    {
      Robot.turret.runBallFeeder(.75);
      Robot.turret.turretBallCount--;
      isFinished = false;
    }
    else {
      Robot.turret.runShooterRPM(_topShooterRPM, _bottomShooterRPM);
      Robot.turret.runBallFeeder(.75);
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
    Robot.turret.runShooterRPM(0, 0);
    //Robot.turret.runBallFeeder(0);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
