package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class resetGyro extends InstantCommand 
{
  public resetGyro() 
  {
    super();
    requires(Robot.drive);
  }

  @Override
  protected void initialize() 
  {
    Robot.drive.resetGyro();
  }
}
