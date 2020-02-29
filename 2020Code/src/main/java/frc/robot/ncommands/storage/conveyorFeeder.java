/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.storage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.Storage;

public class conveyorFeeder extends CommandBase {
  /**
   * Creates a new conveyorFeeder.
   */
  public Storage storage; 
  public int time; 
  boolean isFinished;
  public conveyorFeeder(Storage _storage, int _time) 
  {
    storage = _storage;
    time = _time;
    addRequirements(storage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(Timer.getMatchTime() < time)
    {
      storage.runConveyor(.85);
      storage.resetConveyorEncoder();
      RobotContainer.intake.runRoller(-.3);
      isFinished = false;
    }
    else 
    {
      storage.runConveyor(0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    storage.runConveyor(0);
    RobotContainer.intake.runRoller(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
