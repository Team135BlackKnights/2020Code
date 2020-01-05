/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;


import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;





/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ColorWheel extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public ColorSensorV3 sensor; 
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public static ColorWheel instance; 
  public Color detectedColor;
 

  public static ColorWheel initializeColorWheel()
  {
     if(instance == null)
     { instance = new ColorWheel();}
     return instance;
  }
  
  private ColorWheel()

  {
     sensor = new ColorSensorV3(i2cPort);
  }

  public double red()
  {
     return detectedColor.red;
  }

  public double green()
  {
     return detectedColor.green;
  }
  public double blue()
  {
     return detectedColor.blue;
  }

  
  
 
 public int IRdistance()
 {
    return sensor.getProximity();
  }
  public void printOut()
  {
    detectedColor = sensor.getColor();

    SmartDashboard.putNumber("color red",red());
    SmartDashboard.putNumber("color green",green());
    SmartDashboard.putNumber("color blue  ",blue());
    SmartDashboard.putNumber("IR Distance",IRdistance());

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void periodic()
  {
     printOut();
  }
}
