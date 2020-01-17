package frc.robot.commands.drive;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnToAngle extends TimedCommand 
{
	public boolean is360, turnLeft, isFinished;

	public double error, _angle, drivePower, turnModifier, 
	maxPower, basePower, minPower;
	
	double angleError; 


	public TurnToAngle(double angle) 
	{
		super(1);
		requires(Robot.drive);
		//requires(Robot.sensors);
		this._angle = angle;
	}

	// Called once when the command executes
	protected void initialize() 
	{
		SmartDashboard.putNumber("AngletoTurnTO", this._angle);
		SmartDashboard.putString("Command Running","Turn To Angle");

	}

	protected void execute()
    {	
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
	
	protected boolean isFinished() 
	{
		return  (Math.abs(angleError) <= 2) || 
		Robot.oi.GetJoystickZValue(0) >.2;
	}

	protected void interupted() {this.end();}

	protected void end() {
		Robot.drive.stopMotors();
		SmartDashboard.putString("Command Running","No Command Running");
	}

}