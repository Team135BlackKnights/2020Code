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
import frc.robot.util.MotorControl;

public class Storage extends SubsystemBase implements RobotMap.INTAKE{
  public CANSparkMax conveyorSpark;
  public CANEncoder conveyorEncoder;
  public DigitalInput intakeBallTripSwitch;
  public int currentBallCount;
  public boolean lastSwitchPosition;
  public double desiredEncoderPos;

  public Storage() {
    //Conveyor Spark setup
    conveyorSpark = new CANSparkMax(CONVEYOR_SPARK, MotorType.kBrushless);
    MotorControl.initCANSparkMax(conveyorSpark, true, false, 30);

    //Conveyor Encoder setup
    conveyorEncoder = conveyorSpark.getEncoder();
    desiredEncoderPos = -4;

    //Ball Counting setup
    currentBallCount = 0;
    intakeBallTripSwitch = new DigitalInput(INTAKE_TRIP_SWITCH);

    //init completed
    System.out.println("Storage Initialized");
  }
  
  //run the storage conveyor
  public void runConveyor(double power) {
    conveyorSpark.set(power);
  }

  //check if the ball is at the trip switch
  public boolean isBallAtTripSwitch() {
    return intakeBallTripSwitch.get();
  }

  //Reset encoder position if the ball is at the trip switch
  public void autoResetEncoder() {
    if (isBallAtTripSwitch()) {
      MotorControl.resetSparkEncoder(conveyorEncoder);
    }
  }

  //keep active count of number of balls in the storage system
  public void intakeBallCount() {
    //Counts up on state change from False to True by making sure that there was a change and that it then is positive
    if ((isBallAtTripSwitch() != lastSwitchPosition) && (isBallAtTripSwitch())) {
      currentBallCount++;
      RobotContainer.activeBallCount++;
    }
    //update previous update
    if (lastSwitchPosition != isBallAtTripSwitch()) 
      lastSwitchPosition = isBallAtTripSwitch();
  }

  //TODO:: When new shooter is on finish if statements
  public void autoMoveBalls()
  {
    if(isBallAtTripSwitch())// && //!RobotContainer.nTurret.isReadyForBall)
    {
      MotorControl.resetSparkEncoder(conveyorEncoder);
    }

    // Warning was coming from the if() being always true so I commented it out and had it just run
    /*
    if(RobotContainer.nTurret.isReadyForBall)
    {
      runConveyor(.4);;
    }
    */
    runConveyor(.4);

    if(MotorControl.getSparkEncoderRotations(conveyorEncoder) > desiredEncoderPos)
    {
      runConveyor(-.4);
    }
  }

  //Print storage information for debugging purposes 
  public void printStorageData()
  {
    SmartDashboard.putNumber("Balls Through System ", currentBallCount);
    SmartDashboard.putNumber("Conveyor Position", MotorControl.getSparkEncoderPosition(conveyorEncoder));
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
