
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.commands.DriveWithJoysticks;


public class Drivetrain extends Subsystem {
  public static Drivetrain instance;

  public CANSparkMax frontLeftMotor = new CANSparkMax(OI.MOTORS.FRONT_LEFT_SPARK_ID, MotorType.kBrushless);
	public CANSparkMax rearLeftMotor = new CANSparkMax(OI.MOTORS.REAR_LEFT_SPARK_ID, MotorType.kBrushless);
	public CANSparkMax frontRightMotor = new CANSparkMax(OI.MOTORS.FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);
	public CANSparkMax rearRightMotor = new CANSparkMax(OI.MOTORS.REAR_RIGHT_SPARK_ID, MotorType.kBrushless);

  public Drivetrain()
	{
		// Configuring each drive motor to all have the same settings
		ConfigSpark(frontLeftMotor);
		ConfigSpark(frontRightMotor);
		ConfigSpark(rearLeftMotor);
		ConfigSpark(rearRightMotor);
	}
	public void ConfigSpark(CANSparkMax spark)
	{
		spark.setIdleMode(IdleMode.kBrake);
		spark.setInverted(false);
  }
  
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
  }

  public static Drivetrain getInstance() {if (instance == null) {instance = new Drivetrain();}return instance;}

}
