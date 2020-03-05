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
import frc.robot.util.MotorControl;

public class Endgame extends SubsystemBase implements RobotMap.ENDGAME
{
  public CANSparkMax liftRaiseSpark;
  public CANEncoder liftRaiseEncoder;
  public Solenoid rachet; 

  // Sets motors for endgame
  public Endgame()
  {
    liftRaiseSpark = new CANSparkMax(LIFT_UP_SPARK_ID, MotorType.kBrushless);
    liftRaiseEncoder = liftRaiseSpark.getEncoder();
    MotorControl.initCANSparkMax(liftRaiseSpark, IdleMode.kBrake);

    liftRaiseSpark.setInverted(true);
    rachet = new Solenoid(3);

    MotorControl.resetSparkEncoder(liftRaiseEncoder);
  }

  public void setLiftBrakeMode(IdleMode mode)
  {
    liftRaiseSpark.setIdleMode(mode);
  }

  // Runs spark to raise the lift
  public void runLiftRaiseSpark(double power) 
  {
    // Sets power for raise spark
    power = MotorControl.limit(power, .95, -.95);
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

  public double getMotorPower(CANSparkMax spark)
  {
    return spark.getAppliedOutput();
  }

  // Prints the distance traveled by the lift raise to the smart dash
  public void printPosition()
  {
    SmartDashboard.putNumber("LiftRaise Encoder Distance", MotorControl.getSparkEncoderPosition(liftRaiseEncoder));
  }

  // Prints the velocity of the lift raise to the smart dash
  public void printVelocity()
  {
    SmartDashboard.putNumber("LiftRaise Encoder Velocity", MotorControl.getSparkVelocity(liftRaiseEncoder));
  }

  // Checks the temperature of the lift raise motor
  public double getLiftRaiseTemp()
  {
    return liftRaiseSpark.getMotorTemperature();
  }

  // Prints the lift raise motor temperature to the smart dash
  public void printTemp()
  {
    SmartDashboard.putNumber("LiftRaise Motor Temp", getLiftRaiseTemp());
  }

  // Checks the voltage of the lift raise motor
  public double getVoltage(CANSparkMax spark)
  {
    return spark.getBusVoltage();
  }

  // Prints the lift raise motor voltage to the smart dash
  public void printVoltage() 
  {
    SmartDashboard.putNumber("LiftRaise Motor Voltage", getVoltage(liftRaiseSpark));
  }

  // Prints all information
  public void printEverything()
  {
    printPosition();
    printVelocity();
    printTemp();
    printVoltage();


  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduled run
  }
}
