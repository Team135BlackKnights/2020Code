
package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;

//Creates a class for the Joysticks to control the bot
public class DriveWithJoysticks extends Command {
  private double JoystickYValue, RJoystickZValue; //Declares variables to store the positions of different joysticks

  public double halfPowerDrive;
  public boolean swapControls;

  //Method which finds the position of the Joysticks and assigns them to a corresponding value
  public DriveWithJoysticks() {
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time and does nothing
  @Override
  protected void initialize() {
    SmartDashboard.putString("Drive Command Running: ","Drive with Joysticks");

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //Creates a value for each joystick's power based on the direction/power of the joystick and the max set by the slider
    JoystickYValue = Robot.oi.GetJoystickYValue(1) *  Robot.oi.returnRightSlider();
    RJoystickZValue = Robot.oi.GetJoystickZValue(0) * Robot.oi.returnLeftSlider();
    double testJoystickPower = Robot.oi.GetJoystickYValue(2);
    //If the left or right triggers are pulled, the drive speed is set to 75% of normal, else it is 100%
    halfPowerDrive = (OI.leftThumb() ) ? .625 :1;

    //Detects if the swap controls button is being pushed and if controls are not already swapped, and swaps them to tank drive

    boolean isReverseDirection = OI.rightTrigger() || OI.leftTrigger();

    

    double lateralPower, rotationPower;
    
    //Declare the power based off the correct stick and, if it is active, lowered power mode to drive slower.
   
    lateralPower = JoystickYValue * halfPowerDrive;
    rotationPower = RJoystickZValue * halfPowerDrive;
    
   
      if(isReverseDirection){
        Robot.drive.ArcadeDrive(lateralPower, -rotationPower * .85);
      }
      else {
      Robot.drive.ArcadeDrive(-lateralPower, rotationPower * .85);
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
    Robot.drive.stopMotors();
    SmartDashboard.putString("Command Finished: ", "Drive With Joysticks");
    SmartDashboard.putString("Drive Command Running:","No Command Running");


  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
