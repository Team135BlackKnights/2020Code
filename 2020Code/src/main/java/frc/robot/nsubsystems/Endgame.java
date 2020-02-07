/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.RobotMap;

public class Endgame extends SubsystemBase implements RobotMap.ENDGAME{
  
  public CANSparkMax winchSpark, liftRaiseSpark;
  public CANEncoder liftRaiseEncoder, winchEncoder; 
  public Endgame() 
  {
    winchSpark = new CANSparkMax(WIND_UP_SPARK_ID, MotorType.kBrushless);
    liftRaiseSpark = new CANSparkMax(LIFT_UP_SPARK_ID, MotorType.kBrushless);
    liftRaiseEncoder = liftRaiseSpark.getEncoder();
    winchEncoder = winchSpark.getEncoder();
    initCANSparkMax(winchSpark, IdleMode.kBrake);
    initCANSparkMax(liftRaiseSpark, IdleMode.kBrake);

    resetAllEndgameEncoders();
  }

  public void initCANSparkMax(CANSparkMax spark, IdleMode mode)
  {
		spark.setInverted(false);
    spark.enableVoltageCompensation(12);
    spark.setIdleMode(mode);
  }

   
public void runWinchSpark(double power)
{
 winchSpark.set(power);
}

public void runLiftRaiseSpark(double power)
{
  limit(power, .5, -.5);

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
  return ticks/4096;  // NEED TO UPDATE WITH ACTUAL CONVERSION RATE
}
 
public void printPosition()
{
   
   SmartDashboard.putNumber("LiftRaise Encoder Distance", getLiftRaiseEncoderPosition());
   SmartDashboard.putNumber("Winch Encoder Distance", getWinchEncoderPosition());
}
 
public double getLiftEncoderVelocity()
{
  return liftRaiseEncoder.getVelocity();
}
 
public double getWinchEncoderVelocity()
{
  return winchEncoder.getVelocity();
}

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

public double getVoltage(CANSparkMax spark)
{
  return spark.getBusVoltage();
}

public void printVoltage()
{
  SmartDashboard.putNumber("LiftRaise Motor Voltage", getVoltage(liftRaiseSpark));
  SmartDashboard.putNumber("Winch Motor Voltage", getVoltage(winchSpark));
}

public double limit(double x, double upperLimit, double lowerLimit)
	{	if(x >= upperLimit){ x = upperLimit;}
		else if( x<=lowerLimit){ x = lowerLimit;}
		return x;
  }
 
public void printEverything()
{
  printPosition();
  printVelocity();
  printTemp();
  printVoltage();
 
}

  @Override
  public void periodic() {
    printPosition();
    //printTicks();
    // This method will be called once per scheduler run
  }
}
