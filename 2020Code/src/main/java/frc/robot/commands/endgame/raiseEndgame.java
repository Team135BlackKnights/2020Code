/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.endgame;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class raiseEndgame extends TimedCommand {
  /**
   * Add your docs here.
   */
  
  public double _target;
  public double _power;
  public boolean isFinished;
  public raiseEndgame(double timeout, double target, double power) {
    super(1);
    _target = target * 4096;
    _power = power;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    while (Robot.endgame.getLiftRaiseEncoderPosition() < _target)
    {
      Robot.endgame.runLiftRaiseSpark(_power);
    }


  }

  @Override
  protected boolean isFinished() {
  
    return super.isFinished();
  }

  // Called once after timeout
  @Override
  protected void end() {
    Robot.endgame.runLiftRaiseSpark(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
