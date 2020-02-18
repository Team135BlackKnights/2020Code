/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.FalconDrive;

public class turnToAngle extends CommandBase {
  /**
   * Creates a new turnToAngle.
   */
  FalconDrive drive;
  public double desiredAngle;
  public double prevError = 0, error;
  public boolean isFinished; 
  public turnToAngle(FalconDrive _drive, double _desiredAngle)
   {
     drive = _drive;
     desiredAngle= _desiredAngle;
     addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isFinished = false; 
    SmartDashboard.putString("Drive Command Running:", "Turn to Angle: " + desiredAngle);
    error = drive.pose.getRotation().getDegrees()-desiredAngle;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(Math.abs(error) <1)
    {
      isFinished = true; 
    }

    double currentAngle = drive.pose.getRotation().getDegrees();
    double minPower = .22;

    error = currentAngle-desiredAngle;
    double errorSum =+ error*.02;

    double changeInError = (error-prevError)/.02;

    double power = error/90;
    if(error > 0)
    {
      power = drive.limit(power, .75, minPower);
    }
     else if(error <0)
     {
      power = drive.limit(power, -minPower, -.75);

     }
    double kP, kI, kD;

    kP =.75;    kI=0;
    kD = 0;

    double rotationInput = 
    power * kP + errorSum * kI + changeInError *kD;

    drive.ArcadeDrive(0, rotationInput);
    SmartDashboard.putNumber("rotation Input ", rotationInput);

    prevError = error; 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drive.stopMotors();
    SmartDashboard.putString("Drive Command Running:", "No command Running");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished || Math.abs(RobotContainer.leftJoystick.getRawAxis(2)) > .1;
  }
}
