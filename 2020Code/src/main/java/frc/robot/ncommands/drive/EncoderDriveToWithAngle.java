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

public class EncoderDriveToWithAngle extends CommandBase {
  FalconDrive drive;
  private double _leftTarget, _rightTarget, _angleDesired;
  private double leftError, rightError, angleError;
  private boolean _shortDistance;

  public EncoderDriveToWithAngle(FalconDrive subsystem, double leftTarget, double rightTarget, double angleDesired,
      boolean shortDistance) {
    drive = subsystem;
    this._shortDistance = shortDistance;
    this._leftTarget = leftTarget;
    this._rightTarget = rightTarget;
    this._angleDesired = angleDesired;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Drive Command Running:", "Encoder Drive");
    SmartDashboard.putBoolean("Is Encoder Drive Finished", isFinished());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentLeftPos = drive.getLeftPos();
    double currentRightPos = drive.getRightPos();

    SmartDashboard.putNumber("Left Target:", _leftTarget);
    SmartDashboard.putNumber("Right Target:", _rightTarget);

    leftError = currentLeftPos - _leftTarget;
    rightError = currentRightPos - _rightTarget;

    double leftPower, rightPower;
    leftPower = leftError / 55;
    rightPower = rightError / 55;

    SmartDashboard.putNumber("Left Error:", leftError);
    SmartDashboard.putNumber("Right Error:", rightError);

    double currentAngle = drive.getAngle();
    // angleError = Math.abs(_angleDesired - currentAngle);
    angleError = _angleDesired - currentAngle;
    SmartDashboard.putNumber("Angle Error", angleError);

    double anglePower;
    anglePower = angleError / 70;

    double leftMinDirection = leftError > 0 ? 1 : -1;
    double rightMinDirection = rightError > 0 ? 1 : -1;

    double minDrivePower = .40;
    double rightP = 1, leftP = 1;
    double angleP = 1.5;

    if (_shortDistance) {
      minDrivePower = .40;
      rightP = 1.5;
      leftP = 1.5;
      angleP = 1;
    }

    rightPower = (minDrivePower * rightMinDirection) + (rightPower * rightP) + (anglePower * angleP);
    leftPower = (minDrivePower * leftMinDirection) + (leftPower * leftP) + (anglePower * angleP);

    rightPower = drive.limit(rightPower, .8, -.8);
    leftPower = drive.limit(leftPower, .8, -.8);
    drive.TankDrive(-leftPower, rightPower);

    SmartDashboard.putNumber("LeftPower:", leftPower);
    SmartDashboard.putNumber("RightPower:", rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopMotors();
    SmartDashboard.putString("Command Finished: ", "Encoder Drive to With Angle");
    SmartDashboard.putString("Drive Command Running:", "No Command Running");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(leftError) <= 1 && Math.abs(rightError) <= 1) && Math.abs(angleError) <= 3;
  }
}
