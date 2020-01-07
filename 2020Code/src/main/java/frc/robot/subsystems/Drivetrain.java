
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;


public class Drivetrain extends Subsystem implements RobotMap.MOTORS{
  public static Drivetrain instance;

  public CANSparkMax frontLeftSpark = new CANSparkMax(FRONT_LEFT_SPARK_ID, MotorType.kBrushless);
  public CANSparkMax rearLeftSpark = new CANSparkMax(REAR_LEFT_SPARK_ID, MotorType.kBrushless);
  public CANSparkMax frontRightSpark = new CANSparkMax(FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);
  public CANSparkMax rearRightSpark = new CANSparkMax(REAR_RIGHT_SPARK_ID, MotorType.kBrushless);

  DifferentialDrive chassis;
  SpeedControllerGroup leftSide, rightSide;

	
	
  public Drivetrain()
	{
		// Configuring each drive motor to all have the same settings
		ConfigSpark(frontLeftSpark);
		ConfigSpark(frontRightSpark);
		ConfigSpark(rearLeftSpark);
		ConfigSpark(rearRightSpark);


		leftSide = new SpeedControllerGroup(frontLeftSpark, rearLeftSpark);
		rightSide = new SpeedControllerGroup(frontRightSpark, rearRightSpark);
	  

		chassis = new DifferentialDrive(leftSide, rightSide);
		chassis.setSafetyEnabled(false);
	}
	
	public void ConfigSpark(CANSparkMax spark)
	{
		spark.setIdleMode(IdleMode.kBrake);
		spark.setInverted(false);
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

  public static Drivetrain getInstance() {if (instance == null) {instance = new Drivetrain();}return instance;}

}
