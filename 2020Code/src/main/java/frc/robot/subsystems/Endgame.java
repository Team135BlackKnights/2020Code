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
  limSwitch = new DigitalInput(LIMIT_ID);
  winchSpark = new CANSparkMax(WIND_UP_SPARK_ID, MotorType.kBrushless);
  liftRaiseSpark = new CANSparkMax(LIFT_UP_SPARK_ID, MotorType.kBrushless);
  liftRaiseEncoder = liftRaiseSpark.getEncoder();
  winchEncoder = winchSpark.getEncoder();
 
  resetAllEndgameEncoders();
 
}
 
 
public void runWinchSpark(double power)
{
 winchSpark.set(power);
}

public void runLiftRaiseSpark(double power)
{
  liftRaiseSpark.set(power);
}
 
public void resetAllEndgameEncoders()
{
  resetWinchEncoder();
  resetLiftEncoder();
}
 
public void resetWinchEncoder(){
  winchEncoder.setPosition(0);
}
 
public void resetLiftEncoder(){
   liftRaiseEncoder.setPosition(0);
}
 
public double getWinchEncoderPosition()
{
  return winchEncoder.getPosition();
}

public double getLiftRaiseEncoderPosition()
{
  return liftRaiseEncoder.getPosition();
}

public double ticksToRotations(double ticks)
{
  return ticks/4096;
}

public double ticksToInches(double ticks)
{
  return ticks/4096;
}
 
public void printPosition()
{
   
   SmartDashboard.putNumber("LiftRaise Encoder Distance", getLiftRaiseEncoderPosition());
   SmartDashboard.putNumber("Winch Encoder Distance", getWinchEncoderPosition());
}
 /*
public double getLiftEncoderVelocity()
{
  return liftRaiseEncoder.getVelocity();
}
 
public double getWinchEncoderVelocity()
{
  return winchEncoder.getVelocity();
}
*/
public double returnEncoderVelocity(CANEncoder encoder)
{
   return encoder.getVelocity();
}
public double getWinchRPM()
{
  return ticksToRotations(returnEncoderVelocity(winchEncoder));
}

public void printVelocity()
{
  SmartDashboard.putNumber("LiftRaise Encoder Velocity", returnEncoderVelocity(liftRaiseEncoder));
  SmartDashboard.putNumber("Winch Encoder Velocity", returnEncoderVelocity(winchEncoder));
}
 
public double getLiftRaiseTemp()
{
  return liftRaiseSpark.getMotorTemperature();
}
 
public double getWinchTemp()
{
  return winchSpark.getMotorTemperature();
}
 
public void printTemp()
{
  SmartDashboard.putNumber("LiftRaise Motor Temp", getLiftRaiseTemp());
  SmartDashboard.putNumber("Winch Motor Temp", getWinchTemp());
}

public double getLiftRaiseVoltage()
{
  return liftRaiseSpark.getBusVoltage();
}
 
public double getWinchVoltage()
{
  return winchSpark.getBusVoltage();
}

public void printVoltage()
{
  SmartDashboard.putNumber("LiftRaise Motor Voltage", getLiftRaiseVoltage());
  SmartDashboard.putNumber("Winch Motor Voltage", getWinchVoltage());
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
 
 

