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
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.util.ImprovedJoystick;

public class EncoderDrive extends CommandBase {
  FalconDrive drive;
  public double _leftTarget, _rightTarget, _distanceFromWall,
              leftError, rightError, _tolerance;
  public boolean _stopWhenDone;
  public double actualDistanceFromWall;
  private ImprovedJoystick _joystick;
  /**
   * Creates a new EncoderDrive.
   */
  public EncoderDrive(FalconDrive subsystem, double leftTarget, double rightTarget, double tolerance, boolean stopWhenDone, Joystick joystick) {
    drive = subsystem;
    _joystick = new ImprovedJoystick(joystick);
    this._leftTarget = leftTarget;
    this._rightTarget = rightTarget;
    this._tolerance = tolerance;
    this._stopWhenDone = stopWhenDone;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
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

    leftError = currentLeftPos-_leftTarget;
    rightError = currentRightPos - _rightTarget;

    double leftPower, rightPower;
    leftPower = leftError/60;
    rightPower = rightError/60;

    double 
    leftP = 1.4,  rightP = 1.4;
    
    double minDrivePower = .26; 
    double leftMinAlt = leftError > 0 ? 1: -1;
    double rightMinAlt = rightError > 0 ? 1: -1;
    double leftMinPower = minDrivePower * leftMinAlt;
    double rightMinPower = minDrivePower * rightMinAlt;
    

    leftPower = drive.limit(leftPower, .45, -.45);
    rightPower = drive.limit(rightPower, .45, -.45);

    
      leftPower = drive.limit((leftPower *leftP) + leftMinPower, .7, -.7);
      rightPower = drive.limit((rightPower * rightP) +rightMinPower , .7, -.7);
    
    SmartDashboard.putNumber("Encoder Drive Left Power", leftPower);
    SmartDashboard.putNumber("Encoder Drive Right Power", rightPower);

    drive.TankDrive(-leftPower, rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(this._stopWhenDone)
    {
      drive.stopMotors();
    }
    SmartDashboard.putString("Command Finished: ", "Encoder Drive");
    SmartDashboard.putString("Drive Command Running:","No Command Running");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(leftError) <= _tolerance && Math.abs(rightError) <=_tolerance) ||
    _joystick.getJoystickAxis(2) >.2 ;  }
}
