/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.*;

public class DriveWithJoysticks extends CommandBase {
  private final FalconDrive drive;
  private final DoubleSupplier lateralSupply, rotationalSupply;
  private final BooleanSupplier halfSupply, reverseSupply;
  /**
   * Creates a new DriveWithJoysticks.
   */

  public DriveWithJoysticks(FalconDrive subsystem, 
  DoubleSupplier lateral, DoubleSupplier rotational, 
  BooleanSupplier halfPower, BooleanSupplier reversePower
  ) 
  {
    drive = subsystem;
    addRequirements(drive);
    lateralSupply = lateral;
    rotationalSupply = rotational;
    halfSupply = halfPower;
    reverseSupply = reversePower;


    
    // Use addRequirements() here to declare subsystem dependencies.
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
     boolean isReversed, isHalfPower;
     
     //Declare the power based off the correct stick and, if it is active, lowered power mode to drive slower.
    
     lateralPower = lateralSupply.getAsDouble();
     rotationPower = rotationalSupply.getAsDouble();
     isReversed = reverseSupply.getAsBoolean();
     isHalfPower = halfSupply.getAsBoolean();

     if(isHalfPower)
     {
       lateralPower = lateralPower*.75;
       rotationPower = rotationPower * .5;
     }
     
    
       if(isReversed){
         drive.ArcadeDrive(-lateralPower, -rotationPower * .85);
       }
       else {
       drive.ArcadeDrive(lateralPower, rotationPower * .85);
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
