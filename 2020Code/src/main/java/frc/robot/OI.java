
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.auton.EncoderDriveToWithAngle;
import frc.robot.commands.drive.*;
import frc.robot.commands.drive.commandGroups.*;
import frc.robot.commands.prototyping.*;

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
	
	public final static int 
	TOP_POV = 0 ,
	RIGHT_POV = 1, 
	BOTTOM_POV = 2,
	LEFT_POV = 3,
	TOP_RIGHT_POV = 4,
	BOTTOM_RIGHT_POV = 5,
	BOTTOM_LEFT_POV = 6, 
	TOP_LEFT_POV = 7;

  public static Joystick[] joysticks = {leftJoystick, rightJoystick, manipJoystick};

  public static JoystickButton 
	rightTrigger = new JoystickButton(rightJoystick, TRIGGER_BUTTON),
	leftTrigger = new JoystickButton(leftJoystick, TRIGGER_BUTTON),
	swapControlsButton = new JoystickButton(rightJoystick, HANDLE_TOP_RIGHT_BUTTON),
	prototypeButtonOne = new JoystickButton(manipJoystick, TRIGGER_BUTTON),
	prototypeButtonTwo = new JoystickButton(manipJoystick, THUMB_BUTTON),
	resetEncoderButton = new JoystickButton(rightJoystick, THUMB_BUTTON),
	encoderDriveTestButton = new JoystickButton(leftJoystick, TRIGGER_BUTTON),
	autoTestButton = new JoystickButton(leftJoystick, THUMB_BUTTON),
	gyroResetButton = new JoystickButton(leftJoystick, HANDLE_BOTTOM_RIGHT_BUTTON),
	prototypeShooterButton = new JoystickButton(manipJoystick, HANDLE_BOTTOM_LEFT_BUTTON),
	turnToAngle90 = new JoystickButton(rightJoystick, BASE_BOTTOM_RIGHT_BUTTON);

	public OI()
	{
		
		prototypeButtonOne.toggleWhenPressed(new PrototypeButtonControlOne());
		prototypeButtonTwo.toggleWhenPressed(new PrototypeButtonControlTwo());
		turnToAngle90.whenPressed(new TurnToAngle(180));

		prototypeShooterButton.toggleWhenPressed(new PrototypeShooter(1600, 3800));
		resetEncoderButton.whenPressed(new resetDriveEncoders());
		gyroResetButton.whenPressed(new resetGyro());
		encoderDriveTestButton.whenPressed(new EncoderDrive(-50, 50, 2, false));
		autoTestButton.whenPressed(new EncoderDriveToWithAngle(50, -40, 45, false));
		System.out.println("Operator Interface Initialized");

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
public static boolean swapControls()
{
	return swapControlsButton.get();
}

public double getThrottle(Joystick joystick)
{
	return joystick.getThrottle();
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

	public double getPovAngle(Joystick joystick)
	{
		return joystick.getPOV();
	}

	public boolean isPovDirectionPressed(Joystick joystick, int povDirection)
	{
		double povValue = this.getPovAngle(joystick);
		boolean povDirectionPressed = false;
		switch(povDirection) {

		case (TOP_POV):
		if (povValue >= 337.5 || (povValue <= 22.5 && povValue != -1)) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;

		case (TOP_RIGHT_POV):
		if (povValue >= 22.5 && (povValue <= 67.5 )) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
	case (RIGHT_POV):
		if (povValue >= 67.5 && povValue <= 112.5) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;

		case (BOTTOM_RIGHT_POV):
		if (povValue >= 112.5 && povValue <= 157.5) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
	case (BOTTOM_POV):
		if (povValue >= 157.5 && povValue <= 202.5) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
	case (BOTTOM_LEFT_POV):
		if (povValue >= 202.5 && povValue <= 247.5) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
	case (LEFT_POV):
		if (povValue >= 247.5 && povValue <= 337.5) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
		case (TOP_LEFT_POV):
		if (povValue >= 292.5 && povValue <= 292.5) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
	
	}
	return povDirectionPressed;

	}

  
	public static OI getInstance() {if (instance == null) {instance = new OI();}return instance;}
  
}
