
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.color.rotateWheelOfFortune;
//import frc.robot.commands.auton.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.turret.*;
import frc.robot.commands.endgame.*;
import frc.robot.commands.intake.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI implements RobotMap.KOI{
  public static OI instance;
  
  public static Joystick 
    leftJoystick = new Joystick(RobotMap.KOI.LEFT_JOYSTICK),
    rightJoystick = new Joystick(RobotMap.KOI.RIGHT_JOYSTICK),
	manipJoystick = new Joystick(RobotMap.KOI.MANIP_JOYSTICK);
	
	public final int 
	TOP_POV = 0 ,
	RIGHT_POV = 1, 
	BOTTOM_POV = 2,
	LEFT_POV = 3,
	TOP_RIGHT_POV = 4,
	BOTTOM_RIGHT_POV = 5,
	BOTTOM_LEFT_POV = 6, 
	TOP_LEFT_POV = 7,
	LEFT_JOYSTICK = RobotMap.KOI.LEFT_JOYSTICK,
	RIGHT_JOYSTICK = RobotMap.KOI.RIGHT_JOYSTICK,
	MANIP_JOYSTICK = RobotMap.KOI.MANIP_JOYSTICK;

  public static Joystick[] joysticks = {leftJoystick, rightJoystick, manipJoystick};

  public static JoystickButton 
	rightTrigger = new JoystickButton(rightJoystick, TRIGGER_BUTTON),
	rightThumb = new JoystickButton(rightJoystick, THUMB_BUTTON),
	rightButton3 = new JoystickButton(rightJoystick, HANDLE_BOTTOM_LEFT_BUTTON),
	

	leftTrigger = new JoystickButton(leftJoystick, TRIGGER_BUTTON),
	leftThumb = new JoystickButton(leftJoystick, THUMB_BUTTON),
	leftButton11 = new JoystickButton(leftJoystick, BASE_BOTTOM_LEFT_BUTTON),

	manipTrigger = new JoystickButton(manipJoystick, TRIGGER_BUTTON),
	manipThumb = new JoystickButton(manipJoystick, THUMB_BUTTON),
	manipButton3 = new JoystickButton(manipJoystick, HANDLE_BOTTOM_LEFT_BUTTON),
	manipButton5 = new JoystickButton(manipJoystick, HANDLE_BOTTOM_RIGHT_BUTTON),
	manipButton7 = new JoystickButton(manipJoystick, BASE_TOP_LEFT_BUTTON),
	manipButton8 = new JoystickButton(manipJoystick, BASE_TOP_RIGHT_BUTTON),
	manipButton9 = new JoystickButton(manipJoystick, BASE_MIDDLE_LEFT_BUTTON),
	manipButton10 = new JoystickButton(manipJoystick, BASE_MIDDLE_RIGHT_BUTTON),
	manipButton11 = new JoystickButton(manipJoystick, BASE_BOTTOM_LEFT_BUTTON),
	manipButton12 = new JoystickButton(manipJoystick, BASE_BOTTOM_RIGHT_BUTTON);

		

	public OI()
	{

		
		rightButton3.whenPressed(new toggleLight(true));
		initCompControls();
		System.out.println("Operator Interface Initialized");
	}

	public void initCompControls()
	{
		rightButton3.whenPressed(new toggleLight(true));
		
		//leftThumb.whenPressed(new shiftGears(true));
	//	leftButton11.toggleWhenPressed(new toggleCompressor());

		manipTrigger.whileHeld(new runRoller(.9));
		
		manipButton3.whenPressed(new rotateWheelOfFortune(.8));
		//manipButton4.whileHeld(new runRoller(-.8));
		manipButton5.whileHeld(new runWinch(.675));
		manipButton9.whenPressed(new rotateWheelOfFortune(0));
		manipButton10.whenPressed(new raiseEndgame(8));
	//	manipButton11.toggleWhenPressed(new moveIntake(true));
	//	manipButton12.whenPressed(new raiseEndgame(10));
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

public boolean manipTrigger()
{
	return manipTrigger.get();
}

public static boolean leftThumb()
{
	return leftThumb.get();
}

public static boolean rightThumb()
{
	return rightThumb.get();
}
public boolean getManipThumb()
{
	return manipThumb.get();
}

public boolean getManipButton7()
{
	return manipButton7.get();
}

public boolean getManipButton8()
{
	return manipButton8.get();
}

public boolean getButtonOutPut(int joystick, int buttonID)
{
	return joysticks[joystick].getRawButton(buttonID);
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

	public double getPovAngle(int joystickID)
	{
		return joysticks[joystickID].getPOV();
		
	}

	public boolean isPovDirectionPressed(int joystickID, int povDirection)
	{
		double povValue = this.getPovAngle(joystickID);
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
