/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Storage extends SubsystemBase implements RobotMap.INTAKE {

  public CANSparkMax conveyorSpark;
  public CANEncoder conveyorEncoder;
  public DigitalInput intakeBallTripSwitch, testInput;
  public int currentBallCount;
  public boolean lastSwtichPosition;

  public Storage() {
    conveyorSpark = new CANSparkMax(CONVEYOR_SPARK, MotorType.kBrushless);
    initCANSparkMax(conveyorSpark, IdleMode.kBrake);
    conveyorEncoder = conveyorSpark.getEncoder();
    intakeBallTripSwitch = new DigitalInput(INTAKE_TRIP_SWITCH);
    conveyorEncoder = conveyorSpark.getEncoder();
    lastSwtichPosition = false;
    currentBallCount = 0;
    System.out.println("Storage Initialized");
  }

  public void initCANSparkMax(CANSparkMax spark, IdleMode mode) {
    spark.setInverted(false);
    spark.enableVoltageCompensation(12);
    spark.setIdleMode(mode);
  }

  public void runConveyor(double power) {
    conveyorSpark.set(power);
  }

  public double getEncoderPosition(CANEncoder encoder) {
    return encoder.getPosition();
  }

  public double ticksToRotations(double ticks) {
    return ticks / 4096;
  }

  

  public void resetConveyorEncoder() {
    conveyorEncoder.setPosition(0);
  }

  public double getConveyorRotations()
  {
    return getEncoderPosition(conveyorEncoder);
  }

  public double getConveyorVel()
  {
    return conveyorEncoder.getVelocity();
  }

  public boolean isBallAtTripSwitch()
  {
    return intakeBallTripSwitch.get();
  }
  

  public void autoResetEncoder()
  {
    if(!isBallAtTripSwitch())
    {
      resetConveyorEncoder();
    }
  }

  

  public void intakeBallCount()
  {

    if (isBallAtTripSwitch() != lastSwtichPosition && isBallAtTripSwitch() != false) {
      currentBallCount++;
    }
    if (lastSwtichPosition != isBallAtTripSwitch()) {
      lastSwtichPosition = isBallAtTripSwitch();
    }

  }

  public double limit(double x, double upperLimit, double lowerLimit) {
    if (x >= upperLimit) {
      x = upperLimit;
    } else if (x <= lowerLimit) {
      x = lowerLimit;
    }
    return x;
  }

  @Override
  public void periodic() {
    intakeBallCount();
    SmartDashboard.putNumber("Conveyor Position", getEncoderPosition(conveyorEncoder));
    SmartDashboard.putBoolean("is Ball in conveyor", isBallAtTripSwitch());
    autoResetEncoder();

    SmartDashboard.putNumber("Convyeor Vel ", getConveyorVel());
    // This method will be called once per scheduler run
  }
}
