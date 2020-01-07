
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;


public class Drivetrain extends Subsystem {
  public static Drivetrain instance;

  public CANSparkMax frontLeftMotor = new CANSparkMax(RobotMap.MOTORS.FRONT_LEFT_SPARK_ID, MotorType.kBrushless);
	public CANSparkMax rearLeftMotor = new CANSparkMax(RobotMap.MOTORS.REAR_LEFT_SPARK_ID, MotorType.kBrushless);
	public CANSparkMax frontRightMotor = new CANSparkMax(RobotMap.MOTORS.FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);
	public CANSparkMax rearRightMotor = new CANSparkMax(RobotMap.MOTORS.REAR_RIGHT_SPARK_ID, MotorType.kBrushless);

	DifferentialDrive chassis;
	SpeedControllerGroup leftSide, rightSide;

	
	
  public Drivetrain()
	{
		// Configuring each drive motor to all have the same settings
		ConfigSpark(frontLeftMotor);
		ConfigSpark(frontRightMotor);
		ConfigSpark(rearLeftMotor);
		ConfigSpark(rearRightMotor);
		leftSide = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
		rightSide = new SpeedControllerGroup(frontRightMotor, rearRightMotor);
	  

		chassis = new DifferentialDrive(leftSide, rightSide);

	}
	public void ConfigSpark(CANSparkMax spark)
	{
		spark.setIdleMode(IdleMode.kBrake);
		spark.setInverted(false);
  }
  
  public void TankDrive(double leftMotorPower, double rightMotorPower) 
	{
		chassis.tankDrive(leftMotorPower, rightMotorPower);
	}
	public void ArcadeDrive(double y, double z)
  	{
		chassis.arcadeDrive(y, -z);
	}

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
  }

  public static Drivetrain getInstance() {if (instance == null) {instance = new Drivetrain();}return instance;}

}
