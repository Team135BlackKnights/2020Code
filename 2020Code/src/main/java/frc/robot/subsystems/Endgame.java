/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

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


public void runWinch(double power)
{
 winchSpark.set(power);
}

public void liftRaiseSparkRaise(double power){
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

public void getEncoderPosition()
{
  
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

