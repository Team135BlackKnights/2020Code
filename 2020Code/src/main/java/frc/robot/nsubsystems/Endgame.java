/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.RobotMap;

public class Endgame extends SubsystemBase implements RobotMap.ENDGAME {

  public CANSparkMax liftRaiseSpark;
  public CANEncoder liftRaiseEncoder;
  public Solenoid rachet; 

  public Endgame() {
    liftRaiseSpark = new CANSparkMax(LIFT_UP_SPARK_ID, MotorType.kBrushless);
    liftRaiseEncoder = liftRaiseSpark.getEncoder();
    initCANSparkMax(liftRaiseSpark, IdleMode.kBrake);
    liftRaiseSpark.setInverted(true);
    rachet = new Solenoid(3);

    resetAllEndgameEncoders();
  }

  public void initCANSparkMax(CANSparkMax spark, IdleMode mode) {
    spark.setInverted(false);
    spark.enableVoltageCompensation(12);
    spark.setIdleMode(mode);
  }

  public void setLiftBrakeMode(IdleMode mode)
  {
    liftRaiseSpark.setIdleMode(mode);
  }

  public void runLiftRaiseSpark(double power) 
  {
    power = limit(power, .95, -.95);
    liftRaiseSpark.set(power);
  }

  public void setRachetState(boolean isExtended)
  {
    rachet.set(isExtended);
  }

  public boolean getRachetState()
  {
    return rachet.get();
  }

  public void resetAllEndgameEncoders() {
    resetLiftEncoder();
  }


  public void resetLiftEncoder() {
    liftRaiseEncoder.setPosition(0);
  }

  public double getLiftRaiseEncoderPosition() {
    return liftRaiseEncoder.getPosition();
  }

  public double ticksToRotations(double ticks) {
    return ticks / 4096;
  }

  public double ticksToInches(double ticks) {
    return ticks / 4096; // NEED TO UPDATE WITH ACTUAL CONVERSION RATE
  }

  public double getMotorPower(CANSparkMax spark)
  {
    return spark.getAppliedOutput();
  }


  public void printPosition() {

    SmartDashboard.putNumber("LiftRaise Encoder Distance", getLiftRaiseEncoderPosition());
  }

  public double getLiftEncoderVelocity() {
    return liftRaiseEncoder.getVelocity();
  }

  public double returnEncoderVelocity(CANEncoder encoder) {
    return encoder.getVelocity();
  }

 
  public void printVelocity() {
    SmartDashboard.putNumber("LiftRaise Encoder Velocity", returnEncoderVelocity(liftRaiseEncoder));
  }

  public double getLiftRaiseTemp() {
    return liftRaiseSpark.getMotorTemperature();
  }

  public void printTemp() {
    SmartDashboard.putNumber("LiftRaise Motor Temp", getLiftRaiseTemp());
  }

  public double getVoltage(CANSparkMax spark) {
    return spark.getBusVoltage();
  }

  public void printVoltage() 
  {
    SmartDashboard.putNumber("LiftRaise Motor Voltage", getVoltage(liftRaiseSpark));
  }

  public double limit(double x, double upperLimit, double lowerLimit) {
    if (x >= upperLimit) {
      x = upperLimit;
    } else if (x <= lowerLimit) {
      x = lowerLimit;
    }
    return x;
  }

  public void printEverything() {
    printPosition();
    printVelocity();
    printTemp();
    printVoltage();


  }

  @Override
  public void periodic() {
 
    // This method will be called once per scheduler run
  }
}
