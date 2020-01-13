
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;


public class Drivetrain extends Subsystem implements RobotMap.MOTORS{
  public static Drivetrain instance;
// Actual driveTrain will have Falcons
  public CANSparkMax frontLeftSpark = new CANSparkMax(FRONT_LEFT_SPARK_ID, MotorType.kBrushless);
  public CANSparkMax rearLeftSpark = new CANSparkMax(REAR_LEFT_SPARK_ID, MotorType.kBrushless);
  public CANSparkMax frontRightSpark = new CANSparkMax(FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);
  public CANSparkMax rearRightSpark = new CANSparkMax(REAR_RIGHT_SPARK_ID, MotorType.kBrushless);

  DifferentialDrive chassis;
  SpeedControllerGroup leftSide, rightSide;

  public CANEncoder leftEncoder, rightEncoder; 

	//current auto points
	// 0,0 -54,54 -70,70 -91,88 -43,40 -34,49 -76,91 -72,71
	
  public Drivetrain()
	{
		// Configuring each drive motor to all have the same settings
		ConfigSpark(frontLeftSpark);
		ConfigSpark(frontRightSpark);
		ConfigSpark(rearLeftSpark);
		ConfigSpark(rearRightSpark);

		rearLeftSpark.follow(frontLeftSpark);
		rearRightSpark.follow(frontRightSpark);

		leftEncoder = new CANEncoder(frontLeftSpark);
		rightEncoder = new CANEncoder(frontRightSpark);

		chassis = new DifferentialDrive(frontLeftSpark, frontRightSpark);
		chassis.setSafetyEnabled(false);
		resetEncoders();
		SmartDashboard.putBoolean("DriveTrain initialized",true);
	}
	
	public void ConfigSpark(CANSparkMax spark)
	{
		spark.setIdleMode(IdleMode.kBrake);
		spark.setInverted(false);
		spark.enableVoltageCompensation(12);
		
	}
	public double MotorPower(CANSparkMax spark)
	{
		 return spark.getAppliedOutput();
	}
	public void resetEncoders()
	{ 
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);
	}
	public double getEncoderPosition(CANEncoder encoder)
	{ 
		return encoder.getPosition();
	}
	public double getLeftPos()
	{ 
		return getEncoderPosition(leftEncoder);
	}
	public double getRightPos()
	{ 
		return getEncoderPosition(rightEncoder);
	}

	public double getEncoderVelocity(CANEncoder encoder)
	{ 
		return encoder.getVelocity();
	}

	public double getLeftVel()
	{ 
		return getEncoderVelocity(leftEncoder);
	}
	public double getRightVel()
	{
		return getEncoderVelocity(rightEncoder);
	}

	public double limit(double x, double upperLimit, double lowerLimit)
	{	if(x >= upperLimit){ x = upperLimit;}
		else if( x<=lowerLimit){ x = lowerLimit;}
		return x;
	}
	public double applyDeadband(double input, double deadband)
	{
		if(Math.abs(input) <= deadband) input = 0;

		return input; 
	}


//A mode of driving based off setting power to the left and right side of the robot.
    public void TankDrive(double leftMotorPower, double rightMotorPower) 
	{
		chassis.tankDrive(leftMotorPower, rightMotorPower);
	}
//Runs based off power from one joystick and angle from anohter
	public void ArcadeDrive(double y, double z)
  	{
		chassis.arcadeDrive(y, -z);
	}
	
//Simply set the both sides of the robot to have a power of zero
	public void stopMotors()
	{
		TankDrive(0,0);
	}

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
  }
  public void periodic()
  {
	SmartDashboard.putNumber("Front Left Drive Power", MotorPower(frontLeftSpark));
	SmartDashboard.putNumber("Rear Left Drive Power", MotorPower(rearLeftSpark));
	SmartDashboard.putNumber("Front Right Drive Power", MotorPower(frontRightSpark));
	SmartDashboard.putNumber("Rear Right Drive Power", MotorPower(rearRightSpark));

	SmartDashboard.putNumber("left Encoder Position", getEncoderPosition(leftEncoder));
	SmartDashboard.putNumber("right Encoder Positoin" ,getEncoderPosition(rightEncoder));
  }

  public static Drivetrain getInstance() {if (instance == null) {instance = new Drivetrain();}return instance;}

}
