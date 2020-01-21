/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;


import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class EncoderDriveWithSonar extends TimedCommand {
  public double _leftTarget, _rightTarget, _distanceFromWall,
              leftError, rightError, _tolerance;
  public double actualDistanceFromWall;
  public boolean _stopWhenDone;


  
  public EncoderDriveWithSonar(double leftTarget, double rightTarget, double tolerance, double distanceFromWall, boolean stopWhenDone) {
    super(1);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drive);

    this._leftTarget = leftTarget;
    this._rightTarget = rightTarget;
    this._tolerance = tolerance;
    this._distanceFromWall = distanceFromWall;
    this._stopWhenDone = stopWhenDone;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("Drive Command Running:", "Encoder Drive");
    SmartDashboard.putBoolean("Is Encoder Drive Finished", isFinished());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentLeftPos = Robot.drive.getLeftPos();
    double currentRightPos = Robot.drive.getRightPos();

    leftError = currentLeftPos-_leftTarget;
    rightError = currentRightPos - _rightTarget;

    double leftPower, rightPower;
    leftPower = leftError/60;
    rightPower = rightError/60;

    double 
    leftP = 1.5,  rightP = 1.5;
    
    double 
    angleP = .3;   

    actualDistanceFromWall = sonarDistance(Robot.drive.rightSonar);


    double minDrivePower = .26; 
    double leftMinAlt = leftError > 0 ? 1: -1;
    double rightMinAlt = rightError > 0 ? 1: -1;
    double leftMinPower = minDrivePower * leftMinAlt;
    double rightMinPower = minDrivePower * rightMinAlt;
    
    double wallDistancePower = ((actualDistanceFromWall - _distanceFromWall)/_distanceFromWall) * angleP;
    
    if (wallDistancePower < 0) {
      minDrivePower = -minDrivePower;
    }

    leftPower = Robot.drive.limit(leftPower, .45, -.45);
    rightPower = Robot.drive.limit(rightPower, .45, -.45);
  
    SmartDashboard.putNumber("wallDistancePower", wallDistancePower);
      leftPower = Robot.drive.limit(((leftPower *leftP) + leftMinPower) + (wallDistancePower), .8, -.8);
      rightPower = Robot.drive.limit(((rightPower*rightP) + rightMinPower) + (wallDistancePower),.8,-.8);
    
    SmartDashboard.putNumber("Encoder Drive Left Power", leftPower);
    SmartDashboard.putNumber("Encoder Drive Right Power", rightPower);

    Robot.drive.TankDrive(-leftPower, rightPower);
  }

  public double sonarDistance(Ultrasonic sonar)
  {
    return sonar.getRangeInches();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(leftError) <= _tolerance && Math.abs(rightError) <=_tolerance) && 
    ((actualDistanceFromWall + 1 <= _distanceFromWall || actualDistanceFromWall - 1 <= _distanceFromWall)) ||
    Robot.oi.GetJoystickZValue(0) >.2 ;  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if(this._stopWhenDone)
    {
      Robot.drive.stopMotors();
    }
    SmartDashboard.putString("Command Finished: ", "Encoder Drive with Sonar");
    SmartDashboard.putString("Drive Command Running:","No Command Running");

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
