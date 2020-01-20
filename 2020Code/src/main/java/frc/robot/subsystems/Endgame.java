/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */

public class Endgame extends Subsystem implements RobotMap.ENDGAME{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

public DigitalInput limSwitch;
public CANSparkMax windUpSpark, liftRaiseSpark; 

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
  windUpSpark = new CANSparkMax(WIND_UP_SPARK_ID, MotorType.kBrushless);
  liftRaiseSpark = new CANSparkMax(LIFT_UP_SPARK_ID, MotorType.kBrushless);
}

public static int windrotations = 5;
public static int currentrotations = 0;
public void runWinch(double power)
{
  currentrotations = 0;
  while (currentrotations < windrotations)
    windUpSpark.set(power);
  windUpSpark.set(0);
}
public static boolean liftUpButton(){
  liftUpButton = OI.endgameLiftUp.get();
  return liftUpButton;
}

public static int liftrotations = 10;

public void liftRaiseSparkRaise(double power){
  currentrotations = 0;
  while (currentrotations < liftrotations){
  liftRaiseSpark.set(power);
  }
  liftRaiseSpark.set(0);
}

public void liftRaiseSparkLower(double power){
  currentrotations = 0;
  while (currentrotations < liftrotations)
  {
    liftRaiseSpark.set(power * -1);
  }
  liftRaiseSpark.set(0);
}

public void findRotations()
{
  liftRaiseSpark.getselectedsensorposition()
}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

