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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase implements RobotMap.TURRET {

  public WPI_TalonSRX tiltTalon;
  public Relay targetingLight;
  public CANSparkMax rotationSpark, bottomShooterSpark, topShooterSpark, ballFeederSpark;
  public CANEncoder bottomShooterEncoder, topShooterEncoder, ballFeederEncoder;
  public DigitalInput turretBallTripSwitch, forwardRotationLimit, reverseRotationLimit;
  public Encoder rotationEncoder;
  public Counter turretLidar;

  public int turretBallCount = 0;

  public boolean lastBallState = false;

  public Turret() {
    targetingLight = new Relay(TARGETING_LIGHT);
    tiltTalon = new WPI_TalonSRX(TILT_TALON_ID);
    rotationSpark = new CANSparkMax(ROTATION_SPARK_ID, MotorType.kBrushless);
    bottomShooterSpark = new CANSparkMax(BOTTOM_SHOOTER_SPARK_ID, MotorType.kBrushless);
    topShooterSpark = new CANSparkMax(TOP_SHOOTER_SPARK_ID, MotorType.kBrushless);
    ballFeederSpark = new CANSparkMax(FEEDER_SPARK_ID, MotorType.kBrushless);

    turretBallTripSwitch = new DigitalInput(TRIP_SWITCH_ID);
    forwardRotationLimit = new DigitalInput(LEFT_LIMIT_ID);
    reverseRotationLimit = new DigitalInput(RIGHT_LIMIT_ID);

    initTalonSRX(tiltTalon);

    initCANSparkMax(rotationSpark, IdleMode.kBrake);
    initCANSparkMax(bottomShooterSpark, IdleMode.kCoast);
    initCANSparkMax(topShooterSpark, IdleMode.kCoast);
    initCANSparkMax(ballFeederSpark, IdleMode.kBrake);

    topShooterSpark.setInverted(true);
    bottomShooterEncoder = bottomShooterSpark.getEncoder();
    topShooterEncoder = topShooterSpark.getEncoder();
    ballFeederEncoder = ballFeederSpark.getEncoder();
    tiltTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rotationEncoder = new Encoder(ROTATION_ENCODER_A, ROTATION_ENCODER_B);

    rotationEncoder.setDistancePerPulse(64);

    resetAllTurretEncoders();

    System.out.println("Turret Initialized");
  }

  /*
   * public void turretCountBalls() { if(isBallInShooter()!=lastBallState &&
   * isBallInShooter()!=false) { turretBallCount++; }
   * if(lastBallState!=isBallInShooter()) { lastBallState = isBallInShooter(); }
   * 
   * }
   */
  public void initCANSparkMax(CANSparkMax spark, IdleMode mode) {
    spark.restoreFactoryDefaults();
    spark.setInverted(false);
    spark.enableVoltageCompensation(12);
    spark.setIdleMode(mode);
  }

  public void initTalonSRX(WPI_TalonSRX talon) {
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

  public void setLight(boolean on) {
    if (on) {
      targetingLight.set(Value.kForward);
    } else {
      targetingLight.set(Value.kOn);
    }
  }

  public void resetTiltEncoder() {
    tiltTalon.setSelectedSensorPosition(0);
  }

  public void resetShooterEncoders() {
    bottomShooterEncoder.setPosition(0);
    topShooterEncoder.setPosition(0);
  }

  public void resetBallFeederEncoder() {
    ballFeederEncoder.setPosition(0);
  }

  public void resetRotationEncoder() {
    rotationEncoder.reset();
  }

  public void resetAllTurretEncoders() {
    // resetTiltEncoder();
    resetShooterEncoders();
    resetBallFeederEncoder();
    // resetRotationEncoder();;
  }

  public void runTilt(double power) {
    power = limit(power, .9, -.9);
    tiltTalon.set(ControlMode.PercentOutput, power);
  }

  public void runRotation(double power)
  {
    
    if(getForwardRotationLimit())
    {
      power =  limit(power, 0, -.45);
    }
    else if(getReverseRotationLimit())
    {
      power = limit(power, .45, 0);
    }
    else 
    {
    
    power = limit(power, .45, -.45);
    }

    SmartDashboard.putNumber("Rotation Power", power);
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

  public boolean isBallInShooter() {
    return false;// turretBallTripSwitch.get();
  }

  public boolean getForwardRotationLimit() {
    return forwardRotationLimit.get();
  }

  public boolean getReverseRotationLimit() {
    return reverseRotationLimit.get();
  }

  public void autoResetRotation() {
    if (getForwardRotationLimit()) {
      resetRotationEncoder();
    }
  }

  public void autoResetTiltEncoder() {
    if (tiltTalon.isRevLimitSwitchClosed() == 1) {
      tiltTalon.setSelectedSensorPosition(0);
    }
  }

  public double getSparkEncoderPosition(CANEncoder encoder) {
    return encoder.getPosition();
  }

  public double getRotationTicks() {
    return rotationEncoder.getDistance() / 64;
  }

  public double getRotationRate() {
    return rotationEncoder.getRate();
  }

  public double tiltTicksToAngle() {
    return 180 - ((getTalonPosition(tiltTalon) * 360 / 256) + 90);
  }

  public double getSparkEncoderVelocity(CANEncoder encoder) {
    return encoder.getVelocity();
  }

  public double getTalonPosition(WPI_TalonSRX talon) {
    return talon.getSelectedSensorPosition();
  }

  public double getTalonVelocity(WPI_TalonSRX talon) {
    return talon.getSelectedSensorVelocity();
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

  public void printRotations() {
    SmartDashboard.putNumber("top Shooter Spark Position:",
        ticksToRotations(getSparkEncoderPosition(topShooterEncoder), 4096));
    SmartDashboard.putNumber("bottom Shooter Spark Position:",
        ticksToRotations(getSparkEncoderPosition(bottomShooterEncoder), 4096));
    SmartDashboard.putNumber("ball Feeder Spark Position:",
        ticksToRotations(getSparkEncoderPosition(ballFeederEncoder), 4096));
    SmartDashboard.putNumber("rotation  Position:", getRotationTicks());
    SmartDashboard.putNumber("tilt talon Position:", ticksToRotations(getTalonPosition(tiltTalon), 64));
  }

  public void printStates() {
    SmartDashboard.putBoolean("is Ball in Turret:", isBallInShooter());
    SmartDashboard.putBoolean("is Rotation at Forward limit", getForwardRotationLimit());
    SmartDashboard.putBoolean("is Rotation at Reverse limit", getReverseRotationLimit());
  }

  public void printTiltPos() {
    SmartDashboard.putNumber("TIlt ticks", getTalonPosition(tiltTalon));
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
    SmartDashboard.putNumber("tilt talon power:", getTalonPower(tiltTalon));
  }

  public void printTemp() {
    SmartDashboard.putNumber("top Shooter Spark temp:", getSparkTemp(topShooterSpark));
    SmartDashboard.putNumber("bottom Shooter Spark temp:", getSparkTemp(bottomShooterSpark));
    SmartDashboard.putNumber("ball Feeder Spark temp:", getSparkTemp(ballFeederSpark));
    SmartDashboard.putNumber("rotation spark temp:", getSparkTemp(rotationSpark));
    SmartDashboard.putNumber("tilt talon temp:", getTalonTemp(tiltTalon));
  }

  public void stopShooter() {
    topShooterSpark.set(0);
    bottomShooterSpark.set(0);
  }

  public void stopTurret() {
    rotationSpark.set(0);
    tiltTalon.set(ControlMode.PercentOutput, 0);
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

  public void autoResetEncoders() {
    autoResetRotation();
    autoResetTiltEncoder();
  }

  @Override
  public void periodic() {
   //printTemp();
   autoResetEncoders();
  /* SmartDashboard.putNumber("Rotation Ticks", getRotationTicks());
   SmartDashboard.putNumber("Rotation Ticks With Conversion",getRotationTicks() * (360/512));
   printRotations();
   printStates();
   printTiltPos();
   */
  printRotations();
  printShooterRPM();
    // This method will be called once per scheduler run
  }
}
