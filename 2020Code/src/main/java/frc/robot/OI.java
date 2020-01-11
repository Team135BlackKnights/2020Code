
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI implements RobotMap.KOI{
  public static OI instance;
  
  
  public static Joystick 
    leftJoystick = new Joystick(LEFT_JOYSTICK),
    rightJoystick = new Joystick(RIGHT_JOYSTICK),
    manipJoystick = new Joystick(MANIP_JOYSTICK);

  public static Joystick[] joysticks = {leftJoystick, rightJoystick, manipJoystick};

  public static JoystickButton 
	rightTrigger = new JoystickButton(rightJoystick, TRIGGER_BUTTON),
	leftTrigger = new JoystickButton(leftJoystick, TRIGGER_BUTTON),
	prototypeButtonOne = new JoystickButton(manipJoystick, 1),
	prototypeButtonTwo = new JoystickButton(manipJoystick, 2),
	resetEncoderButton = new JoystickButton(rightJoystick, 2),
	encoderDriveTestButton = new JoystickButton(leftJoystick, 1),
	autoTestButton = new JoystickButton(leftJoystick, 2);

	public OI()
	{
		prototypeButtonOne.toggleWhenPressed(new PrototypeButtonControlOne());
		prototypeButtonTwo.toggleWhenPressed(new PrototypeButtonControlTwo());
		//resetEncoderButton.whenPressed(new resetEncoders());
		encoderDriveTestButton.whenPressed(new EncoderDrive(-50, 50,4));
		//autoTestButton.whenPressed(new AutoMaybe());
	}
//Returns the values for the sliders of the three joysticks 
// adding a deadband so that it doesn't count .01 as still doing something
    public double returnManipSlider() {
		return (-((Math.abs(manipJoystick.getRawAxis(3)) < JOYSTICK_DEADBAND) ?
			0 : manipJoystick.getRawAxis(3)) + 1) / 2;
	}
    public double returnLeftSlider() {
		return (-((Math.abs(leftJoystick.getRawAxis(3)) < JOYSTICK_DEADBAND) ? 
			0 : leftJoystick.getRawAxis(3)) + 1) / 2;
	}
	public double returnRightSlider() {
		return (-((Math.abs(rightJoystick.getRawAxis(3)) < JOYSTICK_DEADBAND) ? 
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
		return (Math.abs(joystickValue) < JOYSTICK_DEADBAND ? 0.0 : joystickValue);
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
