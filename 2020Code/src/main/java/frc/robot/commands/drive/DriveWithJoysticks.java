
package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

//Creates a class for the Joysticks to control the bot
public class DriveWithJoysticks extends Command {
  private double JoystickYValue, RJoystickYValue, RJoystickZValue; //Declares variables to store the positions of different joysticks

  public double halfPowerDrive;
  public boolean swapControls;

  //Method which finds the position of the Joysticks and assigns them to a corresponding value
  public DriveWithJoysticks() {
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time and does nothing
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //Creates a value for each joystick's power based on the direction/power of the joystick and the max set by the slider
    JoystickYValue = Robot.oi.GetJoystickYValue(1) *  Robot.oi.returnRightSlider();
    RJoystickYValue = Robot.oi.GetJoystickYValue(0) * Robot.oi.returnLeftSlider();
    RJoystickZValue = Robot.oi.GetJoystickZValue(0) * Robot.oi.returnLeftSlider();

    //If the left or right triggers are pulled, the drive speed is set to 75% of normal, else it is 100%
    halfPowerDrive = (OI.rightTrigger() || OI.leftTrigger() ) ? .75 :1;

    //Detects if the swap controls button is being pushed and if controls are not already swapped, and swaps them to tank drive
    swapControls= (OI.swapControls() && swapControls ==false) ? true : false;


    double leftDrivePower, rightDrivePower, lateralPower, rotationPower;
    
    //Declare the power based off the correct stick and, if it is active, lowered power mode to drive slower.
    leftDrivePower = JoystickYValue* halfPowerDrive;
    rightDrivePower = RJoystickYValue * halfPowerDrive;
    lateralPower = JoystickYValue * halfPowerDrive;
    rotationPower = RJoystickZValue * halfPowerDrive;
    
    //If swapControls is true, it uses TankDrive rather than the default Arcade Drive
    if(swapControls)
    {
      Robot.drive.TankDrive(leftDrivePower, rightDrivePower);
    }
    else {
      Robot.drive.ArcadeDrive(lateralPower, rotationPower * .85);
    }

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
