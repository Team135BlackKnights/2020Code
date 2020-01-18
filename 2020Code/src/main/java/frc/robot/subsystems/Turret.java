/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
/**
 * Add your docs here.
 */
public class Turret extends Subsystem implements RobotMap.TURRET{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_TalonSRX tiltTalon; 
  public CANSparkMax rotationSpark, bottomShooterSpark, topShooterSpark, ballFeederSpark;
  public DigitalInput turretBallTripSwitch, turretLeftLimit, turretRightLimit, turretTiltLimit;
  public Counter turretLidar; 
  public PigeonIMU pidgey; 


  public static Turret instance; 

  public static Turret getInstance() {
  if(instance == null) { instance = new Turret(); }
  return instance;}


  public Turret()
  {
    tiltTalon = new WPI_TalonSRX(TILT_TALON_ID);
    rotationSpark = new CANSparkMax(ROTATION_SPARK_ID, MotorType.kBrushless);
    bottomShooterSpark = new CANSparkMax(BOTTOM_SHOOTER_SPARK_ID, MotorType.kBrushless);
    topShooterSpark = new CANSparkMax(TOP_SHOOTER_SPARK_ID, MotorType.kBrushless);
    ballFeederSpark = new CANSparkMax(FEEDER_SPARK_ID, MotorType.kBrushless);

    turretBallTripSwitch = new DigitalInput(TRIP_SWITCH_ID);
    turretLeftLimit = new DigitalInput(LEFT_LIMIT_ID);
    turretRightLimit = new DigitalInput(RIGHT_LIMIT_ID);
    turretTiltLimit = new DigitalInput(TILT_LIMIT_ID);

    pidgey = new PigeonIMU(PIGEON_ID);
    initTalonSRX(tiltTalon);

    initCANSparkMax(rotationSpark);
    initCANSparkMax(bottomShooterSpark);
    initCANSparkMax(topShooterSpark);
    initCANSparkMax(ballFeederSpark);


  }

  public void initCANSparkMax(CANSparkMax spark)
  {
		spark.setInverted(false);
		spark.enableVoltageCompensation(12);
  }

  public void initTalonSRX(WPI_TalonSRX talon)
  {
    talon.configReverseSoftLimitEnable(false);
    talon.configForwardSoftLimitEnable(false);
   // talon.enableCurrentLimit(true);
    talon.enableVoltageCompensation(true);
  }

  public void runTilt(double power)
  {
    
    if(isAtTiltLimit())
    {
      limit(power, 0, -.9);
    }

    tiltTalon.set(ControlMode.PercentOutput, power);

  }

  public void runRotation(double power)
  {
   // rotateTurret(-.75);
    /*
    if(Robot.limelight.GetLimelightData()[1] > 1 && Robot.limelight.GetLimelightData()[0] >= 1)
		{
			rotateTurret(.75);
			SmartDashboard.putString("Direction turret turning:", "left");
      SmartDashboard.putString("Turret Not moving", "false");

    if(isAtLeftLimit())
    {
      limit(power, .9, 0);
    }
    else if (isAtRightLimit()){
      limit(power,0, -.9);
    }
    else 
    {
      limit(power, .9, -.9);
    }

    rotationSpark.set(power);
*/
  }
  
  public void runTopShooter(double power)
  {
    topShooterSpark.set(power);
  }

  public void runBottomShooter(double power)
  {
    bottomShooterSpark.set(power);
  }

  public void runBallFeeder(double power)
  {
    ballFeederSpark.set(power);
  }

  public boolean isBallInShooter()
  {
    return turretBallTripSwitch.get();
  }

  public boolean isAtLeftLimit()
  {
    return turretLeftLimit.get();
  }

  public boolean isAtRightLimit()
  {
    return turretRightLimit.get();
  }

  public boolean isAtTiltLimit()
  {
    return turretTiltLimit.get();
  }

  public double limit(double x, double upperLimit, double lowerLimit)
	{	if(x >= upperLimit){ x = upperLimit;}
		else if( x<=lowerLimit){ x = lowerLimit;}
		return x;
	}



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
