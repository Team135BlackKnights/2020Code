/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.endgame;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class raiseEndgame extends TimedCommand {
  /**
   * Add your docs here.
   */
  
  public double _target;
  public boolean isFinished;
  public raiseEndgame(double target) {
    super(1);
    _target = target;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    isFinished = false;
    SmartDashboard.putString("Endgame command Running: ", "raise Endgame" + _target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override

    // Todo kill all while loops 
   
  protected void execute() {   

    double targetError = Robot.endgame.getLiftRaiseEncoderPosition() -_target;

    double kP = 1;
    double power = targetError/40;
    double minPower = .25;
    double minDirection = targetError>0 ? 1:-1;

    power = ((minPower * minDirection) + (power * kP));
    Robot.endgame.runLiftRaiseSpark(power);

    if(Math.abs(power) <=2)
    {
      isFinished = true;
    }
    else 
    {
      isFinished = false;
    }

    // \/\/\/ UNKNOWN PURPOSE AND BROKE CODE, SO COMMENTED OUT
   // if(Robot.oi.getManipThumb()
  }

  @Override
  protected boolean isFinished() {
  
    return isFinished;
  }

  @Override
  protected void end() 
  {
    Robot.endgame.runLiftRaiseSpark(0);
    SmartDashboard.putString("Endgame command Running: ", "No command Running");
    SmartDashboard.putString("Command Finished: ", "raise Endgame" + _target);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
