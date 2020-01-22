/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;



public class runConveyorSetPos extends TimedCommand {
  /**
   * Add your docs here.
   */
  public double _targetPos;
  public boolean isFinished;
  public runConveyorSetPos(double targetPos) {
    super(1);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this._targetPos = targetPos;
    this.isFinished = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    SmartDashboard.putString("Intake Command Running: ", " RunConveyorSetPos");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    double currentConveyorPos = Robot.intake.getConveyorRotations();
    if(_targetPos+-2 !=currentConveyorPos)
    {
      Robot.intake.runConveyor(.65);
 isFinished = false;
    }
    else
    {
      Robot.intake.runConveyor(0);
 isFinished = true; 
    }
  }

  @Override
  protected boolean isFinished()
  {
    return isFinished;
  }

  // Called once after timeout
  @Override
  protected void end() 
  {
    Robot.intake.runConveyor(0);
    SmartDashboard.putString("Intake Command Running:", "No command Running");
    SmartDashboard.putString("Command Finished: ", "runConveyorSetToPos" + this._targetPos);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
