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
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class newStorage extends SubsystemBase implements RobotMap.INTAKE{
  public CANSparkMax conveyorSpark;
  public CANEncoder conveyorEncoder;
  public DigitalInput intakeBallTripSwitch;
  public int currentBallCount;
  public boolean lastSwitchPosition;
  public double desiredEncoderPos;

  public newStorage() {
    //Conveyor Spark
    conveyorSpark = new CANSparkMax(CONVEYOR_SPARK, MotorType.kBrushless);
    initCANSparkMax(conveyorSpark, IdleMode.kBrake);
    conveyorSpark.setInverted(true);

    //Conveyor Encoder
    conveyorEncoder = conveyorSpark.getEncoder();
    desiredEncoderPos = 4;

    //Ball Count
    currentBallCount = 0;
    intakeBallTripSwitch = new DigitalInput(INTAKE_TRIP_SWITCH);

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

  public double getEncoderPosition (CANEncoder encoder) {
    return encoder.getPosition();
  }

  public double ticksToRotations (double ticks) {
    return ticks / 4096;
  }

  public void resetConveyorEncoder() {
    conveyorEncoder.setPosition(0);
  }

  public double getConveyorRotations() {
    return getEncoderPosition(conveyorEncoder);
  }

  public double getConveyorVel() {
    return conveyorEncoder.getVelocity();
  }

  public boolean isBallAtTripSwitch() {
    return intakeBallTripSwitch.get();
  }

  public void autoResetEncoder() {
    if (isBallAtTripSwitch()) {
      resetConveyorEncoder();
    }
  }

  public void intakeBallCount() {
    if ((isBallAtTripSwitch() != lastSwitchPosition) && (isBallAtTripSwitch())) {
      currentBallCount++;
      RobotContainer.activeBallCount++;
    }
    if (lastSwitchPosition != isBallAtTripSwitch()) 
      lastSwitchPosition = isBallAtTripSwitch();
  }

  public double limit (double current, double upperLimit, double lowerLimit) {
    if (current >= upperLimit) 
      current = upperLimit;
    else if (current <= lowerLimit) 
      current = lowerLimit;
    return current;
  }

  public void autoMoveBalls()
  {
    if(isBallAtTripSwitch())// && //!RobotContainer.nTurret.isReadyForBall)
    {
      resetConveyorEncoder();
    }
    if(1==1)//RobotContainer.nTurret.isReadyForBall)
    {
      runConveyor(.4);;
    }
    if(getConveyorRotations() > desiredEncoderPos)
    {
      runConveyor(.4);
    }
  }

  public void printStorageData()
  {
    SmartDashboard.putNumber("Balls Through System ", currentBallCount);
    SmartDashboard.putNumber("Conveyor Position", getEncoderPosition(conveyorEncoder));
    SmartDashboard.putBoolean("is Ball at trip Switch", isBallAtTripSwitch());
  }

  @Override
  public void periodic() {
    intakeBallCount();
    autoResetEncoder();
    //autoMoveBalls();
    //printStorageData(); 
  }
}
