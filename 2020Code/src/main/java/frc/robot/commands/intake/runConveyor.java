/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class runConveyor extends Command {
  public runConveyor() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    boolean isBallTrip = Robot.intake.isBallAtTripSwitch();
    double currentConveyorPos = Robot.intake.getConveyorRotations();
    double conveyorPower = 0;
    
    if(isBallTrip)
    {
      Robot.intake.resetConveyorEncoder();
    }

    if(currentConveyorPos <= 7)
      {
        conveyorPower = .65;
      }
      else 
      {
        conveyorPower = 0;
      }
     
      
    Robot.intake.runConveyor(conveyorPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
