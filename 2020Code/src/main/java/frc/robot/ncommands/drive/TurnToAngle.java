/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.nsubsystems.FalconDrive;

public class TurnToAngle extends CommandBase {
  public boolean is360, turnLeft, isFinished;

	public double error, _angle, drivePower, turnModifier, 
	maxPower, basePower, minPower, angleError;
	
  FalconDrive drive;
  
  public TurnToAngle(double angle, FalconDrive subsystem) {
    drive = subsystem;
    addRequirements(drive);
    this._angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("AngletoTurnTO", this._angle);
		SmartDashboard.putString("Drive Command Running:","Turn To Angle");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = Robot.drive.getAngle();
		SmartDashboard.putNumber("current angle",currentAngle);
		angleError = Math.abs(currentAngle - _angle);
		boolean turnLeft  = currentAngle < _angle; 
		double turnModifer = turnLeft ? -1: 1;
		
		double P, I, D;
		P = .52;
		I = .28; 
		D = 0;
	
		double turnPower = turnModifer *(angleError/90* P + I + D*Robot.drive.getRotationRate()/3);
		turnPower = Robot.drive.limit(turnPower, .7,-.7);
		Robot.drive.ArcadeDrive(0, turnPower); 
		SmartDashboard.putNumber("angle error", angleError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drive.stopMotors();
		SmartDashboard.putString("Command Finished: ", "Turn To Angle");
		SmartDashboard.putString("Drive Command Running:","No Command Running");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  (Math.abs(angleError) <= 2) || 
		Robot.oi.GetJoystickZValue(0) >.2;
  }

  protected void interupted() {this.end(true);}

}
