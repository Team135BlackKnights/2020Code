/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class EncoderDrive extends TimedCommand {
  public double _leftTarget, _rightTarget, 
              leftError, rightError, _tolerance;
  public EncoderDrive(double leftTarget, double rightTarget, double tolerance) {
    super(1);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    this._leftTarget = leftTarget;
    this._rightTarget = rightTarget;
    this._tolerance = tolerance;
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
    double currentLeftPos = Robot.drivetrain.getLeftPos();
    double currentRightPos = Robot.drivetrain.getRightPos();

    leftError = currentLeftPos-_leftTarget;
    rightError = currentRightPos - _rightTarget;

    double leftPower, rightPower;
    leftPower = leftError/35;
    rightPower = rightError/35;

    double 
    leftP =2.4,  rightP = 2.4,
    leftI = 0,  rightI = 0,
    leftD = 0,  rightD = 0;
    
    double minDrivePower = .15; 
    double leftMinAlt = leftError > 0 ? 1: -1;
    double rightMinAlt = rightError > 0 ? 1: -1;
    double leftMinPower = minDrivePower * leftMinAlt;
    double rightMinPower = minDrivePower * rightMinAlt;
    leftPower = Robot.drivetrain.limit((leftPower *leftP) + leftMinPower, .7, -.7);
    rightPower = Robot.drivetrain.limit((rightPower * rightP) +rightMinPower , .7, -.7);

    SmartDashboard.putNumber("Encoder Drive Left Power", leftPower);
    SmartDashboard.putNumber("Encoder Drive Right Power", rightPower);

    Robot.drivetrain.TankDrive(-leftPower, rightPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(leftError) <= _tolerance && Math.abs(rightError) <=_tolerance) ||
    Robot.oi.GetJoystickZValue(0) >.2;  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Robot.drivetrain.stopMotors();
    SmartDashboard.putString("Command Running:", "No Command Running");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
