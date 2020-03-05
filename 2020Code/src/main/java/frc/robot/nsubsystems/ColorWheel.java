/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.util.MotorControl;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class ColorWheel extends SubsystemBase implements RobotMap.CONTROL_PANEL {

   public ColorSensorV3 controlPanelColorSensor;
   private final I2C.Port i2cPort = I2C.Port.kOnboard;
   public CANSparkMax rotatorSpark;
   public Solenoid extendSolenoid;

   public static ColorWheel instance;
   public Color detectedColor;
   public String currentColor;
   public int colorChanges = 0;
   public String lastSeenColor = "No Color";
   public String gameData, desiredColor;
   public int wheelRotations = 0;
   public boolean atDesiredRoations;
   public CANSparkMax spark;

   public ColorWheel() {
      // Creates a color sensor in controlPanelColorSensor
      controlPanelColorSensor = new ColorSensorV3(i2cPort);
      // Creates a SparkMax motor controller in rotatorSpark
      rotatorSpark = new CANSparkMax(ROTATOR_ID, MotorType.kBrushless);

      initCANSparkMax(rotatorSpark, IdleMode.kBrake);
      // Gets the data sent by the FMS as to what color we need
      gameData = DriverStation.getInstance().getGameSpecificMessage();

      desiredColor = "No Color"; // Desired color is none

      System.out.println("Color Wheel Initialized"); // Prints to screen

   }

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

   public void initCANSparkMax(CANSparkMax spark, IdleMode mode) {
      spark.setInverted(false);
      spark.enableVoltageCompensation(12);
      spark.setIdleMode(mode);
   }

   public String gameColor() {
      // Checks if the gamedata has arrived yet by making sure its length is greater
      // than 0
      if (gameData.length() > 0) {
         // Takes the Game Specific Data and changes it to a usable form
         switch (gameData.charAt(0)) {
         case 'B':
            desiredColor = "Red";
            break;
         case 'G':
            desiredColor = "Yellow";
            break;
         case 'R':
            desiredColor = "Blue";
            break;
         case 'Y':
            desiredColor = "Green";
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
      if ((checkForColor() != lastSeenColor) && (checkForColor() != "No Color")) {
         // if the current color is not the last seen color and it isn't No Color, add 1
         // to the count
         colorChanges++;
      }
      if (currentColor != "No Color") { // If the current color isn't no color, put it into last seen
         lastSeenColor = currentColor;
      }
   }   

    // To stop the control panel, the motor controller is set to 0 power
   public void stopControlPanel() {
      rotatorSpark.set(0);
   }
   public void moveColorWheel(double power)
   {
      power = MotorControl.limit(power, .85, -.85);
      rotatorSpark.set(power);
   }

   public void printOut() {
      // Sets detected color to the current color seen by the color sensor(An RGB
      // value)
      detectedColor = controlPanelColorSensor.getColor();
      // Sets current color to the color currently seen by the sensor(In string form)
      currentColor = checkForColor();
      // Runs the function to count colors
      countColor();

      // Prints the different color values, the current color, the number of color
      // changes, and the desired color to smart dash
      SmartDashboard.putNumber("color red", red());
      SmartDashboard.putNumber("color green", green());
      SmartDashboard.putNumber("color blue  ", blue());

      SmartDashboard.putString("Current Color", currentColor);
      SmartDashboard.putNumber("ColorChanges:", colorChanges);

      SmartDashboard.putString("Desired Color ", desiredColor);
   }

   // Pulls the game specific message from driver station

   @Override
   public void periodic() {
      detectedColor = controlPanelColorSensor.getColor();
      gameColor();
      SmartDashboard.putString("Desired Color", gameColor());
      // Sets the current color to the current color(String)
      currentColor = checkForColor();
      // Counts color changes
      countColor();

      // Prints to the screen
      SmartDashboard.putString("Current Color", currentColor);
      SmartDashboard.putNumber("ColorChanges:", colorChanges);
      // This method will be called once per scheduler run
   }

}