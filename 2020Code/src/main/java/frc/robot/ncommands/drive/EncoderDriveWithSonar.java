/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.util.ImprovedJoystick;

public class EncoderDriveWithSonar extends CommandBase {
  FalconDrive drive;
  public double _leftTarget, _rightTarget, _distanceFromWall,
              leftError, rightError, _tolerance;
  public double actualDistanceFromWall;
  public boolean _stopWhenDone;
  private ImprovedJoystick _joystick;
  /**
   * Creates a new EncoderDriveWithSonar.
   */
  public EncoderDriveWithSonar(FalconDrive subsystem, double leftTarget, double rightTarget, double tolerance, double distanceFromWall, boolean stopWhenDone, Joystick joystick) {
    drive = subsystem;
    _joystick = new ImprovedJoystick(joystick);
    this._leftTarget = leftTarget;
    this._rightTarget = rightTarget;
    this._tolerance = tolerance;
    this._distanceFromWall = distanceFromWall;
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
    leftP = 1.5,  rightP = 1.5;
    
    double 
    angleP = .3;   

    actualDistanceFromWall = sonarDistance(drive.rearRightSonar);


    double minDrivePower = .26; 
    double leftMinAlt = leftError > 0 ? 1: -1;
    double rightMinAlt = rightError > 0 ? 1: -1;
    double leftMinPower = minDrivePower * leftMinAlt;
    double rightMinPower = minDrivePower * rightMinAlt;
    
    double wallDistancePower = ((actualDistanceFromWall - _distanceFromWall)/_distanceFromWall) * angleP;
    
    if (wallDistancePower < 0) {
      minDrivePower = -minDrivePower;
    }

    leftPower = drive.limit(leftPower, .45, -.45);
    rightPower = drive.limit(rightPower, .45, -.45);
  
    SmartDashboard.putNumber("wallDistancePower", wallDistancePower);
      leftPower = drive.limit(((leftPower *leftP) + leftMinPower) + (wallDistancePower), .8, -.8);
      rightPower = drive.limit(((rightPower*rightP) + rightMinPower) + (wallDistancePower),.8,-.8);
    
    SmartDashboard.putNumber("Encoder Drive Left Power", leftPower);
    SmartDashboard.putNumber("Encoder Drive Right Power", rightPower);

    drive.TankDrive(-leftPower, rightPower);
  }

  public double sonarDistance(Ultrasonic sonar)
  {
    return sonar.getRangeInches();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(leftError) <= _tolerance && Math.abs(rightError) <=_tolerance) && 
    ((actualDistanceFromWall + 1 <= _distanceFromWall || actualDistanceFromWall - 1 <= _distanceFromWall)) ||
    _joystick.getJoystickAxis(2) >.2 ;  }
}
