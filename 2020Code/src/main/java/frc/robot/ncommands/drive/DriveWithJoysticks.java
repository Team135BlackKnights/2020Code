/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.*;
import frc.robot.util.ImprovedJoystick;

public class DriveWithJoysticks extends CommandBase {
  private final FalconDrive drive;
  
  ImprovedJoystick _leftJoystick, _rightJoystick;
  

  public DriveWithJoysticks(FalconDrive subsystem, 
  Joystick leftJoystick, Joystick rightJoystick) 
  {
    drive = subsystem;
    addRequirements(drive);
    _leftJoystick = new ImprovedJoystick(leftJoystick);
    _rightJoystick = new ImprovedJoystick(rightJoystick);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Drive Command Running: ","Drive with Joysticks");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //Creates a value for each joystick's power based on the direction/power of the joystick and the max set by the slider
  
     double lateralPower, rotationPower;
     boolean isReversed = false, isHalfPower;
     
     //Declare the power based off the correct stick and, if it is active, lowered power mode to drive slower.
    
     lateralPower = _rightJoystick.getJoystickAxis(1);
     rotationPower =  _leftJoystick.getJoystickAxis(2);
     isHalfPower = _leftJoystick.getJoystickButtonValue(2);
     isReversed = (_leftJoystick.getJoystickButtonValue(1)||_rightJoystick.getJoystickButtonValue(1));

     if(isHalfPower)
     {
       lateralPower = lateralPower*.75;
       rotationPower = rotationPower * .5;
     }
     
    
       if(isReversed){
         drive.ArcadeDrive(lateralPower, rotationPower * .85);
       }
       else {
       drive.ArcadeDrive(-lateralPower, rotationPower * .85);
       }
 
     }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
