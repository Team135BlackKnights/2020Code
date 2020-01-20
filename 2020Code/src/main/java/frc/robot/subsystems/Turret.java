/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
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
  public CANEncoder rotationEncoder, bottomShooterEncoder, topShooterEncoder, ballFeederEncoder; 
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

    rotationEncoder = rotationSpark.getEncoder(EncoderType.kQuadrature, 4096);
    bottomShooterEncoder = bottomShooterSpark.getEncoder();
    topShooterEncoder = topShooterSpark.getEncoder();
    ballFeederEncoder = ballFeederSpark.getEncoder();
    tiltTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    resetAllTurretEncoders();
    resetPidgey();

    System.out.println("Turret Initialized");
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
    talon.enableVoltageCompensation(true);
    talon.setSensorPhase(true);
    talon.configNominalOutputForward(0);
    talon.configNominalOutputReverse(0);
    talon.configPeakOutputForward(1);
    talon.configPeakOutputReverse(-1);
  }

  public void resetTiltEncoder()
  {
    tiltTalon.setSelectedSensorPosition(0);
  }

  public void resetShooterEncoders()
  {
     bottomShooterEncoder.setPosition(0);
     topShooterEncoder.setPosition(0);
  }
 
  public void resetBallFeederEncoder()
  {
    ballFeederEncoder.setPosition(0);
  }

  public void resetRotationEncoder()
  {
    rotationEncoder.setPosition(0);
  }

  public void resetAllTurretEncoders()
  {
    resetTiltEncoder();
    resetShooterEncoders();
    resetBallFeederEncoder();
    resetRotationEncoder();;
  }

  public void runTilt(double power)
  {
    
    if(isAtTiltLimit())
    {
      limit(power, 0, -.9);
    }
    else 
    {
      limit(power, .9, -.9);
    }

    tiltTalon.set(ControlMode.PercentOutput, power);

  }

  public void runRotation(double power)
  {
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

  }
  
  public void runTopShooter(double power)
  {
    limit(power, .9, -.9);
    topShooterSpark.set(power);
  }

  public void runBottomShooter(double power)
  {
    limit(power, .9, -.9);
    bottomShooterSpark.set(power);
  }

  public void runShootersRPM(double topRPM, double bottomRPM)
  {
    double maxRPM = 5676;
    double _topRPM, _bottomRPM;

    _topRPM = topRPM/maxRPM;
    _bottomRPM = bottomRPM/maxRPM;

    runTopShooter(_topRPM);
    runBottomShooter(_bottomRPM);
  }

  public void runBallFeeder(double power)
  {
    ballFeederSpark.set(power);
  }

  public void aimTurret(double rotationPower, double tiltPower)
  {
     runTilt(tiltPower);
     runRotation(rotationPower);
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

  public double getPigeonHeading()
  {
    return pidgey.getFusedHeading();
  }

  public void resetPidgey()
  {
    pidgey.setFusedHeading(0);
  }

  public double getSparkEncoderPosition(CANEncoder encoder)
  {
    return encoder.getPosition();
  }

  public double getSparkEncoderVelocity(CANEncoder encoder)
  {
    return encoder.getVelocity();
  }
  
  public double getTalonPosition(WPI_TalonSRX talon)
  {
    return talon.getSelectedSensorPosition();
  }

  public double getTalonVelocity(WPI_TalonSRX talon)
  {
     return talon.getSelectedSensorVelocity();
  }

  public double ticksToRotations(double ticks)
  {
    return ticks/4096;
  }

  public double rotationsToInches(double rotations, double wheelDiameter)
  {
    return rotations * wheelDiameter * Math.PI;
  }

  public double getTopWheelRPM()
  {
    return rotationsToInches(getSparkEncoderPosition(topShooterEncoder), TOP_WHEEL_DIAMETER);
  }

  public double getBottomWheelRPM()
  {
    return rotationsToInches(getSparkEncoderPosition(bottomShooterEncoder), BOTTOM_WHEEL_DIAMETER);
  }

  public double getSparkPower(CANSparkMax spark)
  {
     return spark.getAppliedOutput();
  }

  public double getTalonPower(WPI_TalonSRX talon)
  {
    return talon.get();
  }

  public double getSparkTemp(CANSparkMax spark)
  {
    return spark.getMotorTemperature();
  }
  
  public double getTalonTemp(WPI_TalonSRX talon)
  {
    return talon.getTemperature();
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
