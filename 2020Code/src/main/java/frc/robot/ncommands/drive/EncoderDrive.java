/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.FalconDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class encoderDrive extends CommandBase {
  /**
   * Creates a new encoderDrive.
   */
  public FalconDrive drive;
  public double leftDesired, rightDesired, leftError, rightError, prevLeftError, prevRightError;
  public boolean isFinished;
  public encoderDrive(FalconDrive drive, double leftDesired, double rightDesired) 
  {
    this.drive = drive;
    this.leftDesired = leftDesired;
    this.rightDesired = rightDesired;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    SmartDashboard.putString("Drive Command Running:", "Encoder Drive " + leftDesired + rightDesired);

    prevLeftError = 0;
    prevRightError = 0; 
    leftError = leftDesired-drive.getLeftMetres() ;
    rightError = rightDesired-drive.getRightMetres();
    isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    double currentLeftPos = drive.getLeftMetres();
    double currentRightPos = drive.getRightMetres();
    
    double leftError = leftDesired-currentLeftPos;
    double rightError = rightDesired-currentRightPos ;
    SmartDashboard.putNumber("left error ecnoder drive", leftError);
    SmartDashboard.putNumber("right errror encoder drive", rightError);
    
    double leftPower, rightPower;
    

    leftPower = leftError;
    rightPower = rightError;

    double leftErrorSum =+ leftError*.02;
    double rightErrorSum =+ rightError *.02;

    double leftErrorChange = (leftError - prevLeftError)/.02;
    double rightErrorChange = (rightError - prevRightError)/.02;

    double lP, lI, lD, rP, rI, rD; 
    lP = 4.47; lI = 0; lD = 0; 
    rP = 4.47; rI = 0; rD = 0; 
    double leftInput, rightInput; 

    leftInput = leftPower * lP + leftErrorSum * lI + leftErrorChange * lD; 
    rightInput = rightPower * rP + rightErrorSum * rI + rightErrorChange * rD;

    leftInput = drive.limit(leftInput, .65, -.65);
    rightInput = drive.limit(rightInput, .65, -.65);

    prevLeftError = leftError;
    prevRightError = rightError;
    
    this.drive.TankDrive(leftInput, rightInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(leftError) <1 && Math.abs(rightError) < 1;
  }
}
