/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.util.MotorControl;

public class encoderDrive extends CommandBase {
 
  public FalconDrive drive;
  public double leftDesired, rightDesired, leftError, rightError;
  public boolean isFinished;
  public boolean waitingForBalls;

  public encoderDrive(FalconDrive drive, double leftDesired, double rightDesired, boolean waitingForBalls) {
    this.drive = drive;
    this.leftDesired = leftDesired;
    this.rightDesired = rightDesired;
    this.waitingForBalls = waitingForBalls;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Drive Command Running: ", "Encoder Drive " + leftDesired + rightDesired);

    leftError = leftDesired - drive.getLeftMetres();
    rightError = rightDesired - drive.getRightMetres();
    isFinished = false;
    drive.setBrakeMode(NeutralMode.Brake);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If it needs to wait then don't run immediately
    if (waitingForBalls) {
      if (RobotContainer.activeBallCount <= 0)
        waitingForBalls = false;
    }

    // After the wait is done then run
    else {
      // Checks if it is at the location
      if (Math.abs(leftError) < .05 && Math.abs(rightError) < .05) {
        isFinished = true;
      }

      // Get distances for both sides
      double currentLeftPos = drive.getLeftMetres();
      double currentRightPos = drive.getRightMetres();

      // Find distance off our desired
      leftError = leftDesired - currentLeftPos;
      rightError = rightDesired - currentRightPos;

      // Use the error to move proportional to distance
      double lP, rP;
      lP = 3.45;
      rP = 3.45;
      double leftInput, rightInput;
      leftInput = leftError * lP;
      rightInput = rightError * rP;

      // Limit the power to 95%
      leftInput = MotorControl.limit(leftInput, .85, -.85);
      rightInput = MotorControl.limit(rightInput, .85, -.85);

      drive.TankDrive(leftInput, rightInput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Drive Command Running: ", "No Command Running");
    drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

