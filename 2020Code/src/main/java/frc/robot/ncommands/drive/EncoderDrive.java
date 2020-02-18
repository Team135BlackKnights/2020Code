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
    leftError = drive.getLeftMetres() - leftDesired;
    rightError = drive.getRightMetres() - rightDesired;
    isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(Math.abs(leftError) < .2 && Math.abs(rightError) < .2)
    {
      isFinished = true;
    }
    double currentLeftPos = drive.getLeftMetres();
    double currentRightPos = drive.getRightMetres();
    
    double leftError = currentLeftPos - leftDesired;
    double rightError = currentRightPos - rightDesired;
    
    double leftPower, rightPower;

    leftPower = leftError/5;
    rightPower = rightError/5;

    double leftErrorSum =+ leftError*.02;
    double rightErrorSum =+ rightError *.02;

    double leftErrorChange = (leftError - prevLeftError)/.02;
    double rightErrorChange = (rightError - prevRightError)/.02;

    double lP, lI, lD, rP, rI, rD; 
    lP = 1; lI = 0; lD = 0; 
    rP = 1; rI = 0; rD = 0; 
    double leftInput, rightInput; 

    leftInput = leftPower * lP + leftErrorSum * lI + leftErrorChange * lD; 
    rightInput = rightPower * rP + rightErrorSum * rI + rightErrorChange * rD;

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
    return isFinished;
  }
}
