
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.OI;
import frc.robot.RobotMap;

public class ColorWheel extends Subsystem implements RobotMap.CONTROL_PANEL {
   // Put methods for controlling this subsystem
   // here. Call these from Commands.
   public ColorSensorV3 controlPanelColorSensor;
   private final I2C.Port i2cPort = I2C.Port.kOnboard;
   public CANSparkMax rotatorSpark;
   public static ColorWheel instance;
   public Color detectedColor;
   public String currentColor;
   public int colorChanges = 0;
   public String lastSeenColor = "No Color";
   public String gameData, desiredColor;
   public int wheelRotations = 0;
   public boolean atDesiredRoations;
   public WPI_TalonSRX testBoi;
   public CANSparkMax spark;

   // Mins and Maxes declared in RobotMap.CONTROL_PANEL
   // Checks if the color it is seeing is blue by checking it against the min and
   // max tolerances of each color
   public boolean IsBlue() {
      return ((red() >= BlueRedMin && red() <= BlueRedMax) && (green() >= BlueGreenMin && green() <= BlueGreenMax)
            && (blue() >= BlueBlueMin && blue() <= BlueBlueMax));
   }

   // Checks if the color it is seeing is red by checking it against the min and
   // max tolerances of each color

   public boolean IsRed() {
      return ((red() >= RedRedMin && red() <= RedRedMax) && (green() >= RedGreenMin && green() <= RedGreenMax)
            && (blue() >= RedBlueMin && blue() <= RedBlueMax));
   }

   // Checks if the color it is seeing is green by checking it against the min and
   // max tolerances of each color

   public boolean IsGreen() {
      return ((red() >= GreenRedMin && red() <= GreenRedMax) && (green() >= GreenGreenMin && green() <= GreenGreenMax)
            && (blue() >= GreenBlueMin && blue() <= GreenBlueMax));
   }

   // Checks if the color it is seeing is yellow by checking it against the min and
   // max tolerances of each color

   public boolean IsYellow() {
      return ((red() >= YellowRedMin && red() <= YellowRedMax)
            && (green() >= YellowGreenMin && green() <= YellowGreenMax)
            && (blue() >= YellowBlueMin && blue() <= YellowBlueMax));
   }

   // Initialization point
   public ColorWheel()

   {
      // Creates a color sensor in controlPanelColorSensor
      controlPanelColorSensor = new ColorSensorV3(i2cPort);
      // Creates a SparkMax motor controller in rotatorSpark
      rotatorSpark = new CANSparkMax(14, MotorType.kBrushless);
      testBoi = new WPI_TalonSRX(23);
      gameData = DriverStation.getInstance().getGameSpecificMessage(); // Gets the data sent by the FMS as to what color
                                                                       // we need
      desiredColor = "No Color"; // Desired color is none

      System.out.println("Color Wheel Initialized"); // Prints to screen

   }

   public String gameColor() {
      // Checks if the gamedata has arrived yet by making sure its length is greater
      // than 0
      if (gameData.length() > 0) {
         // Takes the Game Specific Data and changes it to a usable form
         switch (gameData.charAt(0)) {
         case 'B':
            desiredColor = "Blue";
            break;
         case 'G':
            desiredColor = "Green";
            break;
         case 'R':
            desiredColor = "Red";
            break;
         case 'Y':
            desiredColor = "Yellow";
            break;
         default:
            desiredColor = "No color";
            break;
         }
      } else {
         desiredColor = "No Color";
         // Code for no data received yet
      }
      return desiredColor;

   }

   // Detect current Red value given from the color sensor
   public double red() {
      return detectedColor.red;
   }

   // Detect current Green value given from the color sensor
   public double green() {
      return detectedColor.green;
   }

   // Detect current Blue value given from the color sensor
   public double blue() {
      return detectedColor.blue;
   }

   // Determine what the current color under the color sensor is
   public String checkForColor() {
      if (IsBlue()) {
         return "Blue";
      } else if (IsRed()) {
         return "Red";
      } else if (IsGreen()) {
         return "Green";
      } else if (IsYellow()) {
         return "Yellow";
      } else
         return "No Color";
   }

   public void countColor() {

      SmartDashboard.putString("Last Seen Color:", lastSeenColor); // Prints the last seen color to dash
      if ((checkForColor() != lastSeenColor) && (checkForColor() != "No Color")) { // if the current color is not the
                                                                                   // last seen color and it isn't No
                                                                                   // Color, add 1 to the count
         colorChanges++;
      }
      if (currentColor != "No Color") { // If the current color isn't no color, put it into last seen
         lastSeenColor = currentColor;
      }
   }

   public String spinToWhatColor(String FMScolor)
   {
      Dictionary colors = new Hashtable<>(); 
  
      colors.put("Red", "Blue"); 
      colors.put("Green", "Yellow"); 
      colors.put("Blue", "Red"); 
      colors.put("Yellow", "Green"); 
      Enumeration values = colors.elements();
      for (Enumeration k = colors.keys(); k.hasMoreElements();) 
        { 
         
            if(k.nextElement() == FMScolor)
            {
               return values.nextElement().toString();
            }
            values.nextElement();
         } 
      return "Error";
   }

   // Takes the desired color and power setting and if the current color isn't the
   // desired color, rotate the panel
   public void getToColor(final String desiredColor, double power) {
      //SmartDashboard.putString("dictval", spinToWhatColor(desiredColor));
      String actualDesired = spinToWhatColor(desiredColor);
      if (checkForColor() != actualDesired) {
         detectedColor = controlPanelColorSensor.getColor();
         SmartDashboard.putString("funcout", checkForColor());

         // Sets the current color to the current color(String)
         currentColor = checkForColor();
         // Counts color changes
         countColor();

         // Prints to the screen
         //SmartDashboard.putString("Current Color", currentColor);
         //SmartDashboard.putNumber("ColorChanges:", colorChanges);
         // rotatorSpark.set(power);
         rotatorSpark.set(power);
      } else {
         rotatorSpark.set(-.8);
         edu.wpi.first.wpilibj.Timer t = new edu.wpi.first.wpilibj.Timer();
         t.start();
         double starttime = t.get();
         while ((t.get() - starttime) < .1 )//runs motor for .25 seconds
         {}
         atDesiredRoations=true;
         stopControlPanel();// Stops the panel
      }
      

   }

   // Takes the desired number of rotations and power setting, and rotates the
   // wheel that many times
   public void rotateColorWheel(double power, double desiredRotations) {
      colorChanges = 0; // Resets number of color changes to 0

      lastSeenColor = checkForColor(); // Sets the last seen color to the current color
      // rotatorSpark.set(power); //Runs the motor at the desired power
      rotatorSpark.set(power);
      while (wheelRotations < desiredRotations) {
         detectedColor = controlPanelColorSensor.getColor();

         // Sets the current color to the current color(String)
         currentColor = checkForColor();
         // Counts color changes
         countColor();

         // Prints to the screen
         if (OI.manipButton4.get())//if this button is pressed the spinning is canceled
         {break;} 
         countColor();
        wheelRotations = colorChanges / 8; //Calculates wheel rotations based on how many color changes its seen
     }
     SmartDashboard.putNumber("Wheel Rotations", wheelRotations );
     SmartDashboard.putNumber("ColorChanges:", colorChanges); //While the number of rotations is less than we need, it continues to count colors and rotate the wheel

     rotatorSpark.set(0);
     wheelRotations = 0;
     atDesiredRoations = true; //After the above function ends, the desired rotations has been reached
  }

  //To stop the control panel, the motor controller is set to 0 power
  public void stopControlPanel()
  {   
     rotatorSpark.set(0);
     testBoi.set(ControlMode.PercentOutput, 0);
  }

   public void printOut()
  {
     //Sets detected color to the current color seen by the color sensor(An RGB value)
    detectedColor = controlPanelColorSensor.getColor();
    //Sets current color to the color currently seen by the sensor(In string form)
    currentColor = checkForColor();
    //Runs the function to count colors
    countColor();

    //Prints the different color values, the current color, the number of color changes, and the desired color to smart dash
    SmartDashboard.putNumber("color red",red());
    SmartDashboard.putNumber("color green",green());
    SmartDashboard.putNumber("color blue  ",blue());
    
    SmartDashboard.putString("Current Color", currentColor );
    SmartDashboard.putNumber("ColorChanges:", colorChanges);

    SmartDashboard.putString("Desired Color ", desiredColor);
  }
  
  //Pulls the game specific message from driver station
  public String gameMessage = DriverStation.getInstance().getGameSpecificMessage();


  @Override
  public void initDefaultCommand() {}
  
  public void periodic() // Periodic function
  {
     //Sets the detected color to the color currently seen by the camera(RGB Val)
   detectedColor = controlPanelColorSensor.getColor();
    
   //Sets the current color to the current color(String)
   currentColor = checkForColor();
   //Counts color changes
   countColor(); 

   //Prints to the screen
   SmartDashboard.putString("Current Color", currentColor );
   SmartDashboard.putNumber("ColorChanges:", colorChanges);
 }

 //Creates the instance colorwheel if the instance is empty
  public static ColorWheel getInstance() {if (instance == null) { instance = new ColorWheel();}return instance;}

}
