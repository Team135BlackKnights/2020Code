/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class EncoderDriveToWithAngle extends TimedCommand {
  /**
   * Add your docs here.
   */
  public double _leftTarget, _rightTarget, _angleDesired;
  public double leftError, rightError, angleError;
  public boolean _shortDistance;

  public EncoderDriveToWithAngle(double leftTarget, double rightTarget, double angleDesired, boolean shortDistance) {
    super(1);
    requires(Robot.drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this._shortDistance = shortDistance;
    this._leftTarget = leftTarget;
    this._rightTarget = rightTarget;
    this._angleDesired = angleDesired;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("Command Running:", "Encoder Drive");
    SmartDashboard.putBoolean("Is Encoder Drive Finished", isFinished());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentLeftPos = Robot.drive.getLeftPos();
    double currentRightPos = Robot.drive.getRightPos();
    
    SmartDashboard.putNumber("Left Target:", _leftTarget );
    SmartDashboard.putNumber("Right Target:", _rightTarget );


    leftError = currentLeftPos - _leftTarget;
    rightError = currentRightPos - _rightTarget;

    double leftPower, rightPower;
    leftPower = leftError/55;
    rightPower = rightError/55;

    SmartDashboard.putNumber("Left Error:", leftError );
    SmartDashboard.putNumber("Right Error:", rightError );



    double currentAngle = Robot.drive.getAngle();
    //angleError = Math.abs(_angleDesired - currentAngle);
    angleError = _angleDesired - currentAngle;
    SmartDashboard.putNumber("Angle Error", angleError);

    double anglePower;
    anglePower = angleError/ 70;


    double leftMinDirection = leftError > 0 ? 1: -1;
    double rightMinDirection = rightError > 0 ? 1: -1;

    double minDrivePower = .40;
    double 
    rightP = 1,  leftP = 1;
    double 
    angleP = 1;

    if (_shortDistance) {
      minDrivePower = .20;
      rightP = 1.5;
      leftP = 1.5;
      angleP = 1;
    }


    rightPower = (minDrivePower * rightMinDirection) + (rightPower * rightP) - (anglePower * angleP);
    leftPower = (minDrivePower * leftMinDirection) + (leftPower * leftP) - (anglePower * angleP);

    rightPower = Robot.drive.limit(rightPower, .8, -.8);
    leftPower = Robot.drive.limit(leftPower, .8, -.8);
    Robot.drive.TankDrive(-leftPower, rightPower);

    SmartDashboard.putNumber("LeftPower:", leftPower);
    SmartDashboard.putNumber("RightPower:", rightPower);
  }
  @Override
  protected boolean isFinished() {
    return (Math.abs(leftError) <= 1 && Math.abs(rightError) <= 1) && Math.abs(angleError) <= 3; }


  // Called once after timeout
  @Override
  protected void end() {
    Robot.drive.stopMotors();
    SmartDashboard.putString("Command Running:", "No Command Running");

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
