/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.util.KnightMath;
import frc.robot.util.MovingAverage;

public class Turret extends SubsystemBase implements RobotMap.TURRET {
 
  public double lastRPM;

  public double[] testPoint1, testPoint2; 
  public double radius;
  public Relay targetingLight;
  public CANSparkMax rotationSpark, tiltSpark, bottomShooterSpark, topShooterSpark, ballFeederSpark;
  public CANEncoder bottomShooterEncoder, topShooterEncoder, ballFeederEncoder, rotationEncoder, tiltEncoder;
  public DigitalInput forwardRotationLimit, reverseRotationLimit, tiltLimit, ballDetector; 
  public boolean currentState, previousState;

  public int turretBallCount = 0;

  public boolean lastBallState = false;
  public boolean isShooterUpToSpeed; 

  public Turret() {
    isShooterUpToSpeed = false; 
    double[] testPoint1 = 
    {
      0,0
    };
    double[] testPoint2 =
    {
      1,1
    };
    double testRadius = KnightMath.radiusFromPoints(testPoint1, testPoint2);
    double[] testCentroid = KnightMath.centroid(testPoint1, testPoint2);
    targetingLight = new Relay(TARGETING_LIGHT);
    tiltSpark  = new CANSparkMax(TILT_SPARK_ID, MotorType.kBrushless);
    rotationSpark = new CANSparkMax(ROTATION_SPARK_ID, MotorType.kBrushless);
    bottomShooterSpark = new CANSparkMax(BOTTOM_SHOOTER_SPARK_ID, MotorType.kBrushless);
    topShooterSpark = new CANSparkMax(TOP_SHOOTER_SPARK_ID, MotorType.kBrushless);
    ballFeederSpark = new CANSparkMax(FEEDER_SPARK_ID, MotorType.kBrushless);

    forwardRotationLimit = new DigitalInput(LEFT_LIMIT_ID);
    reverseRotationLimit = new DigitalInput(RIGHT_LIMIT_ID);
    tiltLimit = new DigitalInput(TILT_LIMIT_ID);
    ballDetector = new DigitalInput(5);
    


    initCANSparkMax(tiltSpark, IdleMode.kBrake);
    initCANSparkMax(rotationSpark, IdleMode.kBrake);
    initCANSparkMax(bottomShooterSpark, IdleMode.kCoast);
    initCANSparkMax(topShooterSpark, IdleMode.kCoast);
    initCANSparkMax(ballFeederSpark, IdleMode.kBrake);

    topShooterSpark.setInverted(true);
    tiltSpark.setInverted(true
    );

    bottomShooterEncoder = bottomShooterSpark.getEncoder();
    topShooterEncoder = topShooterSpark.getEncoder();
    ballFeederEncoder = ballFeederSpark.getEncoder();
    rotationEncoder = rotationSpark.getEncoder();
    tiltEncoder = tiltSpark.getEncoder();

    resetAllTurretEncoders();
    System.out.print(testRadius);
    SmartDashboard.putNumber("test Radius", testRadius);
    SmartDashboard.putNumber("test centroid x ", testCentroid[0]);
    SmartDashboard.putNumber("test centroid y ", testCentroid[1] );
    currentState = ballDetector.get();
    previousState = false;
    System.out.println("Turret Initialized");
  }
  MovingAverage slowCurrent = new MovingAverage(5);

  public void initCANSparkMax(CANSparkMax spark, IdleMode mode) {
    spark.restoreFactoryDefaults();
    spark.setInverted(false);
    spark.enableVoltageCompensation(12);
    spark.setIdleMode(mode);
  }

  public void setLight(boolean on) {
    if (on) {
      targetingLight.set(Value.kForward);
    } else {
      targetingLight.set(Value.kOn);
    }
  }

  public void resetTiltEncoder() 
  {
    tiltEncoder.setPosition(0);
  }

  public void resetShooterEncoders() {
    bottomShooterEncoder.setPosition(0);
    topShooterEncoder.setPosition(0);
  }

  public void resetBallFeederEncoder() {
    ballFeederEncoder.setPosition(0);
  }

  public void resetRotationEncoder() {
    rotationEncoder.setPosition(0);
  }

  public void resetAllTurretEncoders() {
    resetTiltEncoder();
    resetShooterEncoders();
    resetBallFeederEncoder();
    resetRotationEncoder();;
  }

  public void runTilt(double power)
  {
   //
   
   power = limit(power, .5, -.5);
    tiltSpark.set(power);
  }

  public void runRotation(double power)
  {
    /*
    if(getForwardRotationLimit())
    {
      power =  limit(power, 0, -.45);
    }
    else if(getReverseRotationLimit())
    {
      power = limit(power, .45, 0);
    }
    else 
    {*/
    power = limit(power, .45, -.45);
    //}
    rotationSpark.set(power);
  }

  public void runTopShooter(double power) {
    power = limit(power, .9, -.9);
    topShooterSpark.set(power);
  }

  public void runBottomShooter(double power) {
    limit(power, .9, -.9);
    bottomShooterSpark.set(power);
  }

  public void runShooterPower(double topPower, double bottomPower) {
    runTopShooter(topPower);
    runBottomShooter(bottomPower);
  }

  public void runBallFeeder(double power) {
    ballFeederSpark.set(power);
  }

  public void aimTurret(double rotationPower, double tiltPower) {
    runTilt(tiltPower);
    runRotation(rotationPower);
  }
  public boolean getForwardRotationLimit() {
    return forwardRotationLimit.get();
  }

  public boolean getReverseRotationLimit() {
    return reverseRotationLimit.get();
  }

  public boolean getTiltLImit()
  {
    return tiltLimit.get();
  }

  public void autoResetRotation() {
    if (getForwardRotationLimit()) {
      resetRotationEncoder();
    }
  }

  public double getSparkEncoderPosition(CANEncoder encoder) {
    return encoder.getPosition();
  }

  public double getSparkEncoderVelocity(CANEncoder encoder) {
    return encoder.getVelocity();
  }

  public double ticksToRotations(double ticks, double ppr) {
    return ticks / ppr;
  }

  public double rotationsToInches(double rotations, double wheelDiameter) {
    return rotations * wheelDiameter * Math.PI;
  }

  public double getTopWheelRPM() {
    return getSparkEncoderVelocity(topShooterEncoder);
  }

  public double getBottomWheelRPM() {
    return getSparkEncoderVelocity(bottomShooterEncoder);
  }

  public double getFeederRPM() {
    return getSparkEncoderVelocity(ballFeederEncoder);
  }

  public double getSparkPower(CANSparkMax spark) {
    return spark.getAppliedOutput();
  }

  public double getTalonPower(WPI_TalonSRX talon) {
    return talon.get();
  }

  public double getSparkTemp(CANSparkMax spark) {
    return spark.getMotorTemperature();
  }

  public double getTalonTemp(WPI_TalonSRX talon) {
    return talon.getTemperature();
  }

  public double ticksToInches(CANEncoder encoder, double ppr, double WheelDiameter) {
    return rotationsToInches(ticksToRotations(getSparkEncoderPosition(encoder), ppr), WheelDiameter);
  }

  public boolean isBallInTurret()
  {
    return ballDetector.get();
  }

  public void printRotations() {
    SmartDashboard.putNumber("top Shooter Spark Position:",
        ticksToRotations(getSparkEncoderPosition(topShooterEncoder), 4096));
    SmartDashboard.putNumber("bottom Shooter Spark Position:",
        ticksToRotations(getSparkEncoderPosition(bottomShooterEncoder), 4096));
    SmartDashboard.putNumber("ball Feeder Spark Position:",
        ticksToRotations(getSparkEncoderPosition(ballFeederEncoder), 4096));
    SmartDashboard.putNumber("Rotation Encoder Position",
        ticksToRotations(getSparkEncoderPosition(rotationEncoder ), 1));
      }

  public void printStates() {
    SmartDashboard.putBoolean("is turret at Forward limit", getForwardRotationLimit());
    SmartDashboard.putBoolean("is turret at Reverse limit", getReverseRotationLimit());
    SmartDashboard.putBoolean("is turret at Tilt limit", getTiltLImit());
    SmartDashboard.putBoolean("is ball in turret", isBallInTurret());
  }


  public void printShooterRPM() {
    SmartDashboard.putNumber("top shooter RPM", getTopWheelRPM());
    SmartDashboard.putNumber("bottom shooter RPM", getBottomWheelRPM());
    SmartDashboard.putNumber("feeder RPM", getFeederRPM());
  }

  public void printOutputs() {
    SmartDashboard.putNumber("top Shooter Spark power:", getSparkPower(topShooterSpark));
    SmartDashboard.putNumber("bottom Shooter Spark power:", getSparkPower(bottomShooterSpark));
    SmartDashboard.putNumber("ball Feeder Spark power:", getSparkPower(ballFeederSpark));
    SmartDashboard.putNumber("rotation spark power:", getSparkPower(rotationSpark));
    SmartDashboard.putNumber("tilt talon power:", getSparkPower(tiltSpark));
  }

  public void printTemp() {
    SmartDashboard.putNumber("top Shooter Spark temp:", getSparkTemp(topShooterSpark));
    SmartDashboard.putNumber("bottom Shooter Spark temp:", getSparkTemp(bottomShooterSpark));
    SmartDashboard.putNumber("ball Feeder Spark temp:", getSparkTemp(ballFeederSpark));
    SmartDashboard.putNumber("rotation spark temp:", getSparkTemp(rotationSpark));
    SmartDashboard.putNumber("tilt talon temp:", getSparkTemp(rotationSpark));
  }

  public void stopShooter() {
    topShooterSpark.set(0);
    bottomShooterSpark.set(0);
  }

  public void stopTurret() {
    rotationSpark.set(0);
    tiltSpark.set(0);
  }

  public void stopFeeder() {
    ballFeederSpark.set(0);
  }

  public void stopAllTurretMotors() {
    stopShooter();
    stopTurret();
    stopFeeder();
  }

  public double limit(double x, double upperLimit, double lowerLimit) {
    if (x >= upperLimit) {
      x = upperLimit;
    } else if (x <= lowerLimit) {
      x = lowerLimit;
    }
    return x;
  }

  public void autoResetTiltEncoder()
  {
    if(getTiltLImit())
    {
      resetTiltEncoder();
    }
  }

  public void autoResetEncoders() {
    autoResetRotation();
    autoResetTiltEncoder();
  }
  public double topShooterCurrent()
  {
    return topShooterSpark.getOutputCurrent();
  }

  public void UpdateBallCount()
  {
    currentState = isBallInTurret();
    if(previousState && previousState !=currentState)
    {
      RobotContainer.activeBallCount--;
    }
    previousState = currentState;
  }

  @Override
  public void periodic() {
    isBallInTurret();
    printShooterRPM();

    printRotations();
    UpdateBallCount();
    

    SmartDashboard.putNumber("current balls in system ", RobotContainer.activeBallCount);
    SmartDashboard.putBoolean("is ball in turret", isBallInTurret());
    //SmartDashboard.putNumber("ball detectors inches", getBallDetectorInches());
    SmartDashboard.putNumber("topShooterCurrent", topShooterSpark.getOutputCurrent());
   //printTemp();
  // autoResetEncoders();
  /* SmartDashboard.putNumber("Rotation Ticks", getRotationTicks());
   SmartDashboard.putNumber("Rotation Ticks With Conversion",getRotationTicks() * (360/512));
   printRotations();
   printStates();
   printTiltPos();
   */
 // printRotations();
  //printShooterRPM();
    // This method will be called once per scheduler run
  }
}
