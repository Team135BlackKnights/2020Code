
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class DriveWithJoysticks extends Command {
  private double JoystickYValue, RJoystickYValue, RJoystickZValue;

  public double halfPowerDrive;

  public DriveWithJoysticks() {
    requires(Robot.drivetrain);
    
    JoystickYValue = Robot.oi.GetJoystickYValue(1) *  Robot.oi.returnRightSlider();
    RJoystickYValue = Robot.oi.GetJoystickYValue(0) * Robot.oi.returnLeftSlider();
    RJoystickZValue = Robot.oi.GetJoystickZValue(0) * Robot.oi.returnLeftSlider();
    

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {


    JoystickYValue = Robot.oi.GetJoystickYValue(1) *  Robot.oi.returnRightSlider();
    RJoystickYValue = Robot.oi.GetJoystickYValue(0) * Robot.oi.returnLeftSlider();
    RJoystickZValue = Robot.oi.GetJoystickZValue(0) * Robot.oi.returnLeftSlider();

    halfPowerDrive = (OI.rightTrigger() || OI.leftTrigger() ) ? .75 :1;


    double leftDrivePower;
    double rightDrivePower;
    double lateralPower;
    double rotationPower;
    leftDrivePower = JoystickYValue* halfPowerDrive;
    rightDrivePower = RJoystickYValue * halfPowerDrive;
    lateralPower = JoystickYValue * halfPowerDrive;
    rotationPower = RJoystickZValue * halfPowerDrive;
        
    Robot.drivetrain.ArcadeDrive(lateralPower, rotationPower * .85);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
