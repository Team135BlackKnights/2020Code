
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;


public class ColorWheel extends Subsystem implements RobotMap.CONTROL_PANEL{ 
   // Put methods for controlling this subsystem
   // here. Call these from Commands.
   public ColorSensorV3 controlPanelColorSensor;
   private final I2C.Port i2cPort = I2C.Port.kOnboard;
   public CANSparkMax rotatorSpark; 
   public static ColorWheel instance;
   public Color detectedColor;
   public String currentColor;
   public int colorChanges = 0;
   public String lastSeenColor = "Red";
   public String gameData, desiredColor;


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
     rotatorSpark = new CANSparkMax(ROTATOR_ID, MotorType.kBrushless);

     System.out.println("Color Wheel Initialized");
     gameData = DriverStation.getInstance().getGameSpecificMessage();
     desiredColor = "No Color";
     
  }

  public String gameColor()
  {
   if(gameData.length() > 0)
   {
     switch (gameData.charAt(0))
     {
       case 'B' :
         desiredColor = "Blue";
      break;
       case 'G' :
         desiredColor = "Green";
       break;
       case 'R' :
         desiredColor = "Red";
       break;
       case 'Y' :
         desiredColor = "Yellow";
       break;
       default :
         desiredColor = "No color";
       break;
     }
   } else {
      desiredColor = "No Color";
      //Code for no data received yet
   }
   return desiredColor;

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
   //Determine what the current color under the color sensor is
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

  public void countColor() {
   
   SmartDashboard.putString("Last Seen Color:", lastSeenColor);
   if ((checkForColor() != lastSeenColor) && (checkForColor() != "No Color")) {
      colorChanges++;
   }
   if (currentColor != "No Color") {
      lastSeenColor = currentColor;
   }
  }

  public void getToColor(final String desiredColor, double power) {
      if (checkForColor() != desiredColor){
         rotatorSpark.set(power);
      }
      else stopControlPanel();
  }

  public void stopControlPanel()
  {   
     rotatorSpark.set( 0);
  }

   public void printOut()
  {
    detectedColor = controlPanelColorSensor.getColor();
    
    currentColor = checkForColor();
    countColor();

    SmartDashboard.putNumber("color red",red());
    SmartDashboard.putNumber("color green",green());
    SmartDashboard.putNumber("color blue  ",blue());
    
    SmartDashboard.putString("Current Color", currentColor );
    SmartDashboard.putNumber("ColorChanges:", colorChanges);

    SmartDashboard.putString("Desired Color ", desiredColor);

  }
  
  public String gameMessage = DriverStation.getInstance().getGameSpecificMessage();


  @Override
  public void initDefaultCommand() {}
  
  public void periodic()
  {
   detectedColor = controlPanelColorSensor.getColor();
    
   currentColor = checkForColor();
   countColor(); 
   SmartDashboard.putString("Current Color", currentColor );
   SmartDashboard.putNumber("ColorChanges:", colorChanges);
 }

  public static ColorWheel getInstance() {if (instance == null) { instance = new ColorWheel();}return instance;}

}
