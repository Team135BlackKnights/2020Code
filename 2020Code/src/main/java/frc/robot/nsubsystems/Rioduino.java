/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rioduino extends SubsystemBase {
  /**
   * Creates a new Rioduino.
   */
  public static SerialPort rioduino; 
  public static boolean rioduinoConnected = false;
  public static boolean batCritical = false;
  public boolean disableLeds = false;


  public Rioduino() 
  {
    try{
      rioduino = new SerialPort(9600, SerialPort.Port.kMXP);
    
      rioduino.setTimeout(.8);
      clear();
      rioduinoConnected = true;
    }
    catch(Exception e)
    {
      
      DriverStation.getInstance();
      DriverStation.reportWarning("Cannot connect to arduino :(", false);
    }

  }


  private Map<String, String> lastCommand = new HashMap<String, String>() {
    private static final long serialVersionUID = 1L;
      {
          put("pattern", "0");
          put("colour", "MERGE");
          put("delay", "150");
          put("startPoint", "0");
          put("endPoint", "1");
          put("brightness", "100");

      }
    };

    // The command, in a ready state to send.
    protected static String command = "";

    protected static String previousCommand = "E0Z";

     // The number of pixels on one LED strip
     private int pixels = 120;

     // Let's make the colour and command codes
    private Map<String, String> colours = new HashMap<String, String>() {
      private static final long serialVersionUID = 1L;
      {
          put("RED", "16711680");
          put("GREEN", "32768");
          put("YELLOW", "16775680");
          put("PURPLE", "8388736");
          put("ORANGE", "16753920");
          put("BLUE", "255");
          put("VIOLET", "15631086");
          put("MERGE", "6160762");
          put("TAN", "16767411");
          put("PINK", "14027935");
          put("WHITE", "16777215");
          put("TURQUOISE", "65535");
          put("BLACK", "0");
          put("GOLD", "16766720");
          put("SILVER", "12632256");
      }
    };


    public void setAllLedsColor(String color)
    {
      customDisplay(color, 1, 15, 100, 0, pixels);
    }

    public void spaz()
    {
      customDisplay("BLUE" , 4, 50, 100, 0, 1);
    }

    public void batteryInd(double percent, boolean criticalStatus) {

      batCritical = criticalStatus;

      String bColour;
      if (percent <= 0.25)
          bColour = "red";
      else if (percent <= 0.5)
          bColour = "yellow";
      else if (percent <= 0.75)
          bColour = "purple";
      else
          bColour = "green";

      // Use multi-colour display
      customDisplay(bColour, 12, (int) Math.round(percent * 100), 100, 0, 1);
  }

     /**
     * This is used to display a basic pattern on the bling LED lights. Note that for functions
     * above 9, pixelStart and pixelEnd and delay will do nothing.
     * 
     * @param pattern The type of motion or animation pattern you would like to display. Patterns range from 1-12. <p>
     * 1 : Colour wipe <p>
     * 2 : Colour wipe with blank period <p>
     * 3 : Theatre chase<p>
     * 4 : Rainbow<p>
     * 5 : Theatre chase rainbow<p>
     * 6 : Color bar<p>
     * 7 : Color bar flash<p>
     * 8 : Bounce<p>
     * 9 : Bounce wipe<p>
     * 10: Multi bounce<p>
     * 11: Multi bouce wipe<p>
     * 12: Multi colour wipe<p>
     * @param colour Colour, either as a preset such as "RED", "GREEN", "WHITE" (either caps or
     *        no caps) or in decimal format. Use a programmer calculator to determine decimal
     *        format. <p>
     * 
     *        Presets: GREEN, RED, BLUE, YELLOW, ORANGE, PURPLE, TAN, VIOLET, MERGE, PINK, WHITE,
     *        TURQUOISE, BLACK, GOLD, SILVER
     * @param delay The delay between animation segments in seconds, if applicable.
     * @param brightness The brightness of the LED pattern as an integer between 0 and 100.
     * @param pixelStart The percent of the bar where the pixel pattern will start in decimal
     *        format.
     * @param pixelEnd The percent of the bar where the pattern will end in decimal format.
     */




    public void customDisplay(String colour, int pattern, double tDelay, int brightness,
                    int pixelStart, int pixelEnd) {

        int delay = (int) Math.round(tDelay);

        // Get rid of all the spaces
        String gColour = colour.replace(" ", "");

        // Preset colour that we need to convert to proper format
        try {
            Integer.parseInt(gColour);
        } catch (Exception e) {
            // In case the colour is not in the Map.
            try {
                // Make sure that any letters are uppercase.
                gColour = gColour.toUpperCase();
                gColour = colours.get(gColour);
            }

            catch (Exception f) {
                return;
            }
        }

        int startPixel = Math.round(pixelStart * pixels);
        int endPixel = Math.round(pixelEnd * pixels);

        // Record all of what is inputted
        lastCommand.put("pattern", Integer.toString(pattern));
        lastCommand.put("brightness", Integer.toString(brightness));
        lastCommand.put("colour", gColour);
        lastCommand.put("delay", Double.toString(delay));
        lastCommand.put("startPoint", Integer.toString(startPixel));
        lastCommand.put("endPoint", Integer.toString(endPixel));

        if (disableLeds && pattern <= 9)
            pattern = 6;
        else if (disableLeds && pattern > 9)
            pattern = 12;

        String LEDStripRange = "";
        if (pixelStart != 0 || pixelEnd != 1) {
            LEDStripRange = "P" + startPixel + "Q" + endPixel;
        }

        // Let's make it easy for rainbow displaying
        if (pattern == 4) {
            command = ("E4Z");
        }

        else if (pattern <= 9) {
            command = ("F" + pattern + "C" + gColour + "B" + brightness + "D" + delay
                            + LEDStripRange + "E" + pattern + "Z");
        } else {
            command = ("F" + pattern + "C" + gColour + "D" + delay + "B" + brightness + "R10000E"
                            + pattern + "Z");
        }
        send();
    }


    public void send() {
      // IF THE BLINGPORT FAILED, DON'T CAUSE ERRORS
      if (!(previousCommand.equalsIgnoreCase(command)) && rioduinoConnected ) {
          rioduino.writeString("I" + command);
      }

      // Making sure we do not send the same command twice.
      previousCommand = command;
  }

  public void clear() {
    // Clear the LED strip
    command = ("E0Z");
    send();

    lastCommand.put("pattern", "0");
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
