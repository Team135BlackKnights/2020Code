/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
 
/**
 * Add your docs here.
 */
 
public class Endgame extends Subsystem implements RobotMap.ENDGAME{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
 
public DigitalInput limSwitch;
public CANSparkMax winchSpark, liftRaiseSpark;
public CANEncoder liftRaiseEncoder, winchEncoder; 
 
public static Endgame instance;
 
public static Endgame getInstance()
{
  if(instance == null){
    instance = new Endgame();
  } return instance;
}
public static boolean liftUpButton;
 
public Endgame()
{
  // Declares the Limit switch, winch and lift sparks, and their respective encoders
  limSwitch = new DigitalInput(LIMIT_ID);
  winchSpark = new CANSparkMax(WIND_UP_SPARK_ID, MotorType.kBrushless);
  liftRaiseSpark = new CANSparkMax(LIFT_UP_SPARK_ID, MotorType.kBrushless);
  liftRaiseEncoder = liftRaiseSpark.getEncoder();
  winchEncoder = winchSpark.getEncoder();
 
  resetAllEndgameEncoders();
 
}
 
 // Takes a double input and runs the provided spark at the specified power setting

public void runLiftRaiseSpark(double power)
{
  liftRaiseSpark.set(power);
}

public void runWinch(double power)
{
  winchSpark.set(power);
}
 
// Resets both encoders
public void resetAllEndgameEncoders()
{
  resetEncoder(winchEncoder);
  resetEncoder(liftRaiseEncoder);
}
 
// Resets the specified encoder
public void resetEncoder(CANEncoder encoder){
  encoder.setPosition(0);
}
 
// Returns the position of the specified encoder, 
public double getEncoderPosition(CANEncoder encoder)
{
  return encoder.getPosition();
}

public double ticksToRotations(double ticks)
{
  return ticks/4096;
}

public double ticksToInches(double ticks) // NEED TO UPDATE WITH ACTUAL CONVERSION RATE
{
  return ticks/4096;
}

public double getLiftRaiseEncoderPosition()
{
  return getEncoderPosition(liftRaiseEncoder);
}
 
public void printPosition()
{
   
   SmartDashboard.putNumber("LiftRaise Encoder Distance", getEncoderPosition(liftRaiseEncoder));
   SmartDashboard.putNumber("Winch Encoder Distance", getEncoderPosition(winchEncoder));
}

public double getEncoderVelocity(CANEncoder encoder)
{
   return encoder.getVelocity();
}
public double getRPM(CANEncoder encoder)
{
  return ticksToRotations(getEncoderVelocity(encoder));
}

public void printVelocity()
{
  SmartDashboard.putNumber("LiftRaise Encoder Velocity", getEncoderVelocity(liftRaiseEncoder));
  SmartDashboard.putNumber("Winch Encoder Velocity", getEncoderVelocity(winchEncoder));
}
 
public double getMotorTemp(CANSparkMax spark)
{
  return spark.getMotorTemperature();
}
 
public void printTemp()
{
  SmartDashboard.putNumber("LiftRaise Motor Temp", getMotorTemp(liftRaiseSpark));
  SmartDashboard.putNumber("Winch Motor Temp", getMotorTemp(winchSpark));
}

public double getVoltage(CANSparkMax spark)
{
  return spark.getBusVoltage();
}

public void setLiftAndRaisePos(int endgamepos)
{
  
}

public void printVoltage()
{
  SmartDashboard.putNumber("LiftRaise Motor Voltage", getVoltage(liftRaiseSpark));
  SmartDashboard.putNumber("Winch Motor Voltage", getVoltage(winchSpark));
}
 
public void printEverything()
{
  printPosition();
  printVelocity();
  printTemp();
  printVoltage();
 
}
 
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void periodic()
  {
    printEverything();
  }
}
 
 

