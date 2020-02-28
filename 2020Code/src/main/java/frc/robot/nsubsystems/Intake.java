/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase implements RobotMap.INTAKE {

  public CANSparkMax rollerSpark;
  public CANEncoder rollerEncoder;
  public Ultrasonic intakeSonar;
  public Solenoid raiseLower;

  public boolean raiseLowerState;
  public double autonRPM; 
  public Intake() {
    rollerSpark = new CANSparkMax(ROLLER_SPARK, MotorType.kBrushless);

    initCANSparkMax(rollerSpark, IdleMode.kCoast);

    rollerEncoder = rollerSpark.getEncoder();

    raiseLower = new Solenoid(RAISE_LOWER);
    autonRPM = 3500;

    System.out.println("Intake Initialized");
  }

  public void initCANSparkMax(CANSparkMax spark, IdleMode mode)
  {
    spark.setInverted(false);
    spark.enableVoltageCompensation(12);
    spark.setIdleMode(mode);
    spark.setSmartCurrentLimit(30, 30);
  }

  public void runRoller(double power) {
    rollerSpark.set(power);
  }

  public void resetEncoders() {
    rollerEncoder.setPosition(0);
  }

  public double getEncoderPosition(CANEncoder encoder) {
    return encoder.getPosition();
  }

  public double ticksToRotations(double ticks) {
    return ticks / 4096;
  }

  public double getEncoderVelocity(CANEncoder encoder) {
    return encoder.getVelocity();
  }

  public double getIntakeSonarDistanceIn() {
    return intakeSonar.getRangeInches();
  }

  public void raiseLower(boolean position) {
    raiseLower.set(position);
  }

  public boolean isRollerLowered() {
    return raiseLower.get();
  }

  public double getRollerPower() {
    return rollerSpark.getAppliedOutput();
  }
  public double getRollerRPM()
  {
    return rollerEncoder.getVelocity();
  }

  public void printIntakeStuff() {
    SmartDashboard.putBoolean("is Intake Lower", isRollerLowered());
    SmartDashboard.putNumber("intake roller position", getEncoderPosition(rollerEncoder));
    SmartDashboard.putNumber("roller Vel", getRollerRPM());
  }

  @Override
  public void periodic() {
    isRollerLowered();
    SmartDashboard.putNumber("roller Vel", getRollerRPM());

   // printIntakeStuff();

    // This method will be called once per scheduler run
  }
}
