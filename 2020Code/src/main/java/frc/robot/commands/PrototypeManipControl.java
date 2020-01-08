package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PrototypeManipControl extends Command {
  private double ManipJoystickYValue;

  public PrototypeManipControl() {
    requires(Robot.prototyping);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ManipJoystickYValue = Robot.oi.GetJoystickYValue(1);

    Robot.prototyping.runManip(ManipJoystickYValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
