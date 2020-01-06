/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.OI;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class ColorWheel extends Subsystem {
   // Put methods for controlling this subsystem
   // here. Call these from Commands.
   public ColorSensorV3 sensor;
   private final I2C.Port i2cPort = I2C.Port.kOnboard;
   public TalonSRX colorWheelSpinner;
   public static ColorWheel instance;
   public Color detectedColor;

   public ColorWheel initializeColorWheel() {
      if (instance == null) {
         instance = new ColorWheel();
      }
      return instance;
   }

   // Blue color mins and maxes
   public double BlueRedMin = .11;
   public double BlueRedMax = .19;
   public double BlueGreenMin = .42;
   public double BlueGreenMax = .45;
   public double BlueBlueMin = .35;
   public double BlueBlueMax = .46;

   // Green color mins and maxes
   public double GreenRedMin = .15;
   public double GreenRedMax = .2;
   public double GreenGreenMin = .5;
   public double GreenGreenMax = .59;
   public double GreenBlueMin = .25;
   public double GreenBlueMax = .25;

   // Red color mins and maxes
   public double RedRedMin = .38;
   public double RedRedMax = .55;
   public double RedGreenMin = .32;
   public double RedGreenMax = .40;
   public double RedBlueMin = .12;
   public double RedBlueMax = .18;

   // Yellow color mins and maxes
   public double YellowRedMin = .31;
   public double YellowRedMax = .32;
   public double YellowGreenMin = .51;
   public double YellowGreenMax = .56;
   public double YellowBlueMin = .11;
   public double YellowBlueMax = .16;

  

  public boolean IsBlue()
  {
   return ((red() >= BlueRedMin && red()<= BlueRedMax) && (green() >= BlueGreenMin && green() <= BlueGreenMax) && (blue() >= BlueBlueMin && blue() <= BlueBlueMax));
  }

  public boolean IsRed()
  {
   return ((red() >= RedRedMin && red()<= RedRedMax) && (green() >= RedGreenMin && green() <= RedGreenMax) && (blue() >= RedBlueMin && blue() <= RedBlueMax));
  }

  public boolean IsGreen()
  {
   return ((red() >= GreenRedMin && red()<= GreenRedMax) && (green() >= GreenGreenMin && green() <= GreenGreenMax) && (blue() >= GreenBlueMin && blue() <= GreenBlueMax));
  }
  
  public boolean IsYellow() 
  {
   return ((red() >= YellowRedMin && red()<= YellowRedMax) && (green() >= YellowGreenMin && green() <= YellowGreenMax) && (blue() >= YellowBlueMin && blue() <= YellowBlueMax));
  }


  //Initialization point
  private ColorWheel()

  {
     sensor = new ColorSensorV3(i2cPort);
     colorWheelSpinner = new TalonSRX(OI.MOTORS.colorSpinner);

     colorWheelSpinner.setNeutralMode(NeutralMode.Brake);


  }
  
//Detect current Red value given from the color sensor
  public double red()
  {
     return detectedColor.red;
  }
//Detect current Green value given from the color sensor
  public double green()
  {
     return detectedColor.green;
  }
  //Detect current Blue value given from the color sensor
  public double blue()
  {
     return detectedColor.blue;
  }

  
  
 
   public int IRdistance()
   {
    return sensor.getProximity();
   }

   //Determine what the current color under the censor is
   public String checkForColor()
  {
   if (IsBlue()) {
      return "Blue";
   }
   else if (IsRed()){
      return "Red";
   }
   else if (IsGreen()){
      return "Green";
   }
   else if (IsYellow()){
      return "Yellow";
   }
   else
      return null;
  }

  public void getToColor(final String desiredColor, double power) {
      while (checkForColor() != desiredColor){
         colorWheelSpinner.set(ControlMode.PercentOutput, power);
      }
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
   getToColor("Blue", .2);
     
  }
}
