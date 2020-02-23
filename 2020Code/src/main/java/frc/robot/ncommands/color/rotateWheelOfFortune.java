/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.color;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.ColorWheel;

public class rotateWheelOfFortune extends CommandBase {
  ColorWheel colorWheel;
  public boolean isFinished;
  public double manualSpinSpeed;

  /**
   * Creates a new rotateWheelOfFortune.
   */
  public rotateWheelOfFortune(ColorWheel subsystem, double manualSpin) {
    manualSpinSpeed = manualSpin;
    colorWheel = subsystem;
    addRequirements(colorWheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;

    SmartDashboard.putString("Control Panel Command Running:", "rotate Wheel of Fortune");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (manualSpinSpeed != 0)// if = 0, then command is not manual control and skip this block
    {
      colorWheel.rotatorSpark.set(manualSpinSpeed);
      edu.wpi.first.wpilibj.Timer t = new edu.wpi.first.wpilibj.Timer();
      t.start();
      double starttime = t.get();
      while ((t.get() - starttime) < .25)// runs motor for .25 seconds
      {
      }
      colorWheel.rotatorSpark.set(0);
      isFinished = true;
      return;
    }

    // Sets the desired color to the color given by the game
    String DesiredColor = colorWheel.gameColor();
    SmartDashboard.putString("FMS Readout", DesiredColor);
    // If the desired color isn't empty, rotate the wheel at 80% power until it is
    // detected
    if (DesiredColor != "No Color") {
      getToColor(DesiredColor, .25);
      if (colorWheel.atDesiredRoations) { // 2If the wheel has been spun the desired amount, it is finished
        isFinished = true;
        colorWheel.stopControlPanel();
      }
    } 
    else {
      // If the desired color is no color, rotate the wheel four times at 80% power
      rotateColorWheel(.8, 2.75);
      if (colorWheel.checkForColor() != colorWheel.desiredColor) { 
        isFinished = true;
        colorWheel.stopControlPanel();
      }
    }
  }

  public void getToColor(final String desiredColor, double power) {
    // SmartDashboard.putString("dictval", spinToWhatColor(desiredColor));
    String actualDesired = spinToWhatColor(desiredColor);
    SmartDashboard.putString("actual Desired", actualDesired);
    if (colorWheel.checkForColor() != actualDesired) {
       colorWheel.detectedColor = colorWheel.controlPanelColorSensor.getColor();

       // Sets the current color to the current color(String)
       colorWheel.currentColor = colorWheel.checkForColor();
       // Counts color changes
       colorWheel.countColor();
       colorWheel.rotatorSpark.set(power);
    }
    /*
     * else { rotatorSpark.set(-.6); edu.wpi.first.wpilibj.Timer t = new
     * edu.wpi.first.wpilibj.Timer(); t.start(); double starttime = t.get(); while
     * ((t.get() - starttime) < .01 )//runs motor for .25 seconds {}
     * atDesiredRoations=true; stopControlPanel();// Stops the panel }
     */
    else
       colorWheel.stopControlPanel();
 }
 @SuppressWarnings({"unchecked" , "rawtypes"})
   public String spinToWhatColor(String FMScolor) {
      Dictionary colors = new Hashtable<>();
      colors.put("Red", "Blue");
      colors.put("Green", "Yellow");
      colors.put("Blue", "Red");
      colors.put("Yellow", "Green");
      Enumeration values = colors.elements();
      for (Enumeration k = colors.keys(); k.hasMoreElements();) {

         if (k.nextElement() == FMScolor) {
            return values.nextElement().toString();
         }
         values.nextElement();
      }
      return "Error";
   }

   public void rotateColorWheel(double power, double desiredRotations) {
    colorWheel.colorChanges = 0; // Resets number of color changes to 0

    colorWheel.lastSeenColor = colorWheel.checkForColor(); // Sets the last seen color to the current color
    colorWheel.rotatorSpark.set(power);
    
    if(colorWheel.wheelRotations < desiredRotations) {
       colorWheel.detectedColor = colorWheel.controlPanelColorSensor.getColor();
 
       // Sets the current color to the current color(String)
       colorWheel.currentColor = colorWheel.checkForColor();
       // Counts color changes
       colorWheel.countColor();
 
       // Prints to the screen
 
       SmartDashboard.putNumber("Wheel Rotations", colorWheel.wheelRotations);
       SmartDashboard.putNumber("ColorChanges:", colorWheel.colorChanges);
       // countColor();
       colorWheel.wheelRotations = colorWheel.colorChanges / 8; // Calculates wheel rotations based on how many color changes its seen
    }      
    colorWheel.rotatorSpark.set(0);
    colorWheel.wheelRotations = 0;
    colorWheel.atDesiredRoations = true;
    SmartDashboard.putNumber("Wheel Rotations", colorWheel.wheelRotations);
    SmartDashboard.putNumber("ColorChanges:", colorWheel.colorChanges); // While the number of rotations is less than we need, it
                                                             // continues to count colors and rotate the wheel

     // After the above function ends, the desired rotations has been reached
 }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorWheel.stopControlPanel();
    colorWheel.rotatorSpark.set(0);
    SmartDashboard.putString("Control Panel Command Running:", "No command Running");
    colorWheel.atDesiredRoations = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}