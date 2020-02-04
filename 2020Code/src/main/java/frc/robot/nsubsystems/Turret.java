/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase implements RobotMap.TURRET{
  /**
   * Creates a new Turret.
   */
  public WPI_TalonSRX tiltTalon; 
  public Relay targetingLight; 
  public CANSparkMax rotationSpark, bottomShooterSpark, topShooterSpark, ballFeederSpark;
  public CANEncoder rotationEncoder, bottomShooterEncoder, topShooterEncoder, ballFeederEncoder; 
  public CANDigitalInput leftRotationLimit, rightRotationLimit;
  public DigitalInput turretBallTripSwitch;
  public Counter turretLidar; 

  public int turretBallCount = 0; 

  public boolean lastBallState = false;

  public Turret() 
  {
    targetingLight = new Relay(TARGETING_LIGHT);
    tiltTalon = new WPI_TalonSRX(TILT_TALON_ID);
    rotationSpark = new CANSparkMax(ROTATION_SPARK_ID, MotorType.kBrushless);
    bottomShooterSpark = new CANSparkMax(BOTTOM_SHOOTER_SPARK_ID, MotorType.kBrushless);
    topShooterSpark = new CANSparkMax(TOP_SHOOTER_SPARK_ID, MotorType.kBrushless);
    ballFeederSpark = new CANSparkMax(FEEDER_SPARK_ID, MotorType.kBrushless);

    turretBallTripSwitch = new DigitalInput(TRIP_SWITCH_ID);
    
    initTalonSRX(tiltTalon);

    initCANSparkMax(rotationSpark, IdleMode.kBrake);
    initCANSparkMax(bottomShooterSpark, IdleMode.kCoast);
    initCANSparkMax(topShooterSpark, IdleMode.kCoast);
    initCANSparkMax(ballFeederSpark, IdleMode.kBrake);

    topShooterSpark.setInverted(true);
   //rotationEncoder = rotationSpark.getAlternateEncoder();
   rotationEncoder = rotationSpark.getEncoder();
   bottomShooterEncoder = bottomShooterSpark.getEncoder();
    topShooterEncoder = topShooterSpark.getEncoder();
    ballFeederEncoder = ballFeederSpark.getEncoder();
    tiltTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  //  rotationSpark.enableSoftLimit(SoftLimitDirection.kForward, true);
    //rotationSpark.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
   // tiltTalon.configReverseSoftLimitEnable(true);

    resetAllTurretEncoders();
    
    System.out.println("Turret Initialized");
  }

  public void turretCountBalls()
  {
    if(isBallInShooter()!=lastBallState && isBallInShooter()!=false)
    {
      turretBallCount++;
    }
    if(lastBallState!=isBallInShooter())
    {
      lastBallState = isBallInShooter();
    }

  }

  public void initCANSparkMax(CANSparkMax spark, IdleMode mode)
  {
    spark.restoreFactoryDefaults();
		spark.setInverted(false);
    spark.enableVoltageCompensation(12);
    spark.setIdleMode(mode);
  }

  public void initTalonSRX(WPI_TalonSRX talon)
  {
    talon.configReverseSoftLimitEnable(false);
    talon.configForwardSoftLimitEnable(false);
    talon.enableCurrentLimit(false);
    talon.enableVoltageCompensation(true);
    talon.setSensorPhase(true);
    talon.configNominalOutputForward(0);
    talon.configNominalOutputReverse(0);
    talon.configPeakOutputForward(1);
    talon.configPeakOutputReverse(-1);
    talon.setNeutralMode(NeutralMode.Brake);
  }

 
  public void setLight(boolean on)
  {
    if(on)
    {
      targetingLight.set(Value.kForward);
    }
    else {
      targetingLight.set(Value.kOn);
    }
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
    
    limit(power, .9, -.9);
    tiltTalon.set(ControlMode.PercentOutput, power);

  }

  public void runRotation(double power)
  {
    
    limit(power, .9, -.9);

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

  public void runShooterRPM(double topRPM, double bottomRPM)
  {
    double maxRPM = 4500;
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

  
//Tilt is 64 for encoder ticks per revolution
  

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
    return getSparkEncoderVelocity(topShooterEncoder);
  }

  public double getBottomWheelRPM()
  {
    return getSparkEncoderVelocity(bottomShooterEncoder);
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
  public double ticksToInches(CANEncoder encoder, double WheelDiameter) {
    return rotationsToInches(ticksToRotations(getSparkEncoderPosition(encoder)), WheelDiameter);

  }
  public void printRotations()
  {
    SmartDashboard.putNumber("top Shooter Spark Position:", ticksToRotations(getSparkEncoderPosition(topShooterEncoder)));
    SmartDashboard.putNumber("bottom Shooter Spark Position:", ticksToRotations(getSparkEncoderPosition(bottomShooterEncoder)));
    SmartDashboard.putNumber("ball Feeder Spark Position:", ticksToRotations(getSparkEncoderPosition(ballFeederEncoder)));
    SmartDashboard.putNumber("rotation spark Position:", ticksToRotations(getSparkEncoderPosition(rotationEncoder)));
    SmartDashboard.putNumber("tilt talon Position:", ticksToRotations(getTalonPosition(tiltTalon)));
  }

  public void printShooterRPM()
  {
    SmartDashboard.putNumber("top shooter RPM", getTopWheelRPM());
    SmartDashboard.putNumber("bottom shooter RPM", getBottomWheelRPM());
  }

  public void printOutputs()
  {
    SmartDashboard.putNumber("top Shooter Spark power:", getSparkPower(topShooterSpark));
    SmartDashboard.putNumber("bottom Shooter Spark power:", getSparkPower(bottomShooterSpark));
    SmartDashboard.putNumber("ball Feeder Spark power:", getSparkPower(ballFeederSpark));
    SmartDashboard.putNumber("rotation spark power:", getSparkPower(rotationSpark));
    SmartDashboard.putNumber("tilt talon power:", getTalonPower(tiltTalon));
  }

  public void printTemp()
  {
    SmartDashboard.putNumber("top Shooter Spark temp:", getSparkTemp(topShooterSpark));
    SmartDashboard.putNumber("bottom Shooter Spark temp:", getSparkTemp(bottomShooterSpark));
    SmartDashboard.putNumber("ball Feeder Spark temp:", getSparkTemp(ballFeederSpark));
    SmartDashboard.putNumber("rotation spark temp:", getSparkTemp(rotationSpark));
    SmartDashboard.putNumber("tilt talon temp:", getTalonTemp(tiltTalon));
  }

  public void stopShooter()
  {
    topShooterSpark.set(0);
    bottomShooterSpark.set(0);
  }

  public void stopTurret()
  {
    rotationSpark.set(0);
    tiltTalon.set(ControlMode.PercentOutput, 0);
  }
  public void stopFeeder()
  {
    ballFeederSpark.set(0);
  }

  public void stopAllTurretMotors()

  {
    stopShooter();
    stopTurret();
    stopFeeder();
  }
  public double limit(double x, double upperLimit, double lowerLimit)
	{	if(x >= upperLimit){ x = upperLimit;}
		else if( x<=lowerLimit){ x = lowerLimit;}
		return x;
  }

  @Override
  public void periodic() {
   //printTemp();
   printRotations();
    // This method will be called once per scheduler run
  }
}
