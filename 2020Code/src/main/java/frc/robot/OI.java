
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotMap.KOI;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
  public static OI instance;
  

  
  public static Joystick 
    leftJoystick = new Joystick(KOI.LEFT_JOYSTICK),
    rightJoystick = new Joystick(KOI.RIGHT_JOYSTICK),
    manipJoystick = new Joystick(KOI.MANIP_JOYSTICK);

	public static Joystick[] joysticks = {leftJoystick, rightJoystick, manipJoystick};

  public static JoystickButton 
		rightTrigger = new JoystickButton(rightJoystick, KOI.TRIGGER_BUTTON),
		leftTrigger = new JoystickButton(leftJoystick, KOI.TRIGGER_BUTTON);

//Returns the values for the sliders of the three joysticks 
// adding a deadband so that it doesn't count .01 as still doing something
  public double returnManipSlider() {
		return (-((Math.abs(manipJoystick.getRawAxis(3)) < KOI.JOYSTICK_DEADBAND) ?
			0 : manipJoystick.getRawAxis(3)) + 1) / 2;
	}
	public double returnLeftSlider() {
		return (-((Math.abs(leftJoystick.getRawAxis(3)) < KOI.JOYSTICK_DEADBAND) ? 
			0 : leftJoystick.getRawAxis(3)) + 1) / 2;
	}
	public double returnRightSlider() {
		return (-((Math.abs(rightJoystick.getRawAxis(3)) < KOI.JOYSTICK_DEADBAND) ? 
			0 : rightJoystick.getRawAxis(3)) + 1) / 2;
	}
  
//Get value of the triggers for the different joysticks
public static boolean leftTrigger() {
  return leftTrigger.get();
}
public static boolean rightTrigger()
{
  return rightTrigger.get();
}

//returns value of the given joystick with a deadband applied
  private double DeadbandJoystickValue(double joystickValue) {
		return (Math.abs(joystickValue) < KOI.JOYSTICK_DEADBAND ? 0.0 : joystickValue);
	}
	public double GetJoystickYValue(int joystickNumber) {
		return DeadbandJoystickValue( -joysticks[joystickNumber].getY() );
	}
	public double GetJoystickXValue(int joystickNumber) {
		return DeadbandJoystickValue( -joysticks[joystickNumber].getX() );
	}
	public double GetJoystickZValue(int joystickNumber) {
		return DeadbandJoystickValue( -joysticks[joystickNumber].getZ() );
	}

  
	public static OI getInstance() {if (instance == null) {instance = new OI();}return instance;}
  
}
