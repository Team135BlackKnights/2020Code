
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
import frc.robot.RobotMap;


public class ColorWheel extends Subsystem implements RobotMap { 
   // Put methods for controlling this subsystem
   // here. Call these from Commands.
   public ColorSensorV3 controlPanelColorSensor;
   private final I2C.Port i2cPort = I2C.Port.kOnboard;
   public TalonSRX controlPanelTalon;
   public static ColorWheel instance;
   public Color detectedColor;

   public static ColorWheel initializeColorWheel() {
      if (instance == null) {
         instance = new ColorWheel();
      }
      return instance;
   }

   // Blue color mins and maxes
   public double BlueRedMin = .09;
   public double BlueRedMax = .21;
   public double BlueGreenMin = .42;
   public double BlueGreenMax = .50;
   public double BlueBlueMin = .33;
   public double BlueBlueMax = .48;

   // Green color mins and maxes
   public double GreenRedMin = .15;
   public double GreenRedMax = .2;
   public double GreenGreenMin = .5;
   public double GreenGreenMax = .59;
   public double GreenBlueMin = .24;
   public double GreenBlueMax = .27;

   // Red color mins and maxes
   public double RedRedMin = .38;
   public double RedRedMax = .58;
   public double RedGreenMin = .31;
   public double RedGreenMax = .40;
   public double RedBlueMin = .09;
   public double RedBlueMax = .18;

   // Yellow color mins and maxes
   public double YellowRedMin = .29;
   public double YellowRedMax = .35;
   public double YellowGreenMin = .49;
   public double YellowGreenMax = .58;
   public double YellowBlueMin = .10;
   public double YellowBlueMax = .17;

  

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
     controlPanelColorSensor = new ColorSensorV3(i2cPort);
     controlPanelTalon = new TalonSRX(MOTORS.SPINNER_TALON_ID);

     controlPanelTalon.setNeutralMode(NeutralMode.Brake);


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
      return "No Color";
  }

  public void getToColor(final String desiredColor, double power) {
      if (checkForColor() != desiredColor){
         controlPanelTalon.set(ControlMode.PercentOutput, power);
      }
      else controlPanelTalon.set(ControlMode.PercentOutput, 0);

  }

   public void printOut()
  {
    detectedColor = controlPanelColorSensor.getColor();

    SmartDashboard.putNumber("color red",red());
    SmartDashboard.putNumber("color green",green());
    SmartDashboard.putNumber("color blue  ",blue());

    SmartDashboard.putString("Current Color", checkForColor() );

  }

  @Override
  public void initDefaultCommand() {
    
  }
  public void periodic()
  {
   printOut();
   getToColor("Yellow", 1);
     
  }
}
