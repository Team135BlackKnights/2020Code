package frc.robot.util;

/**
 * Add your docs here.
 */
import edu.wpi.first.wpilibj.Joystick;


public class KeatonJoystick {
    public Joystick _joystick;
    public double _deadband;

    public final int 
	TOP_POV = 0 ,
	RIGHT_POV = 1, 
	BOTTOM_POV = 2,
	LEFT_POV = 3,
	TOP_RIGHT_POV = 4,
	BOTTOM_RIGHT_POV = 5,
	BOTTOM_LEFT_POV = 6, 
    TOP_LEFT_POV = 7;

    public KeatonJoystick(Joystick joystick, double deadband)
    {
        this._joystick = joystick;
        this._deadband = deadband;
    }
    public KeatonJoystick(Joystick joystick)
    {
        this._joystick = joystick;
        this._deadband = .2;
    }

    public double getJoystickSlider()
    {
        return (-((Math.abs(_joystick.getRawAxis(3)) < _deadband) ?
		0 : _joystick.getRawAxis(3)) + 1) / 2;
    }

    public double getJoystickAxis(int axis)
    {
        return (Math.abs(_joystick.getRawAxis(axis)) < _deadband ? 0.0 :_joystick.getRawAxis(axis));
    }

    public double getPovAngle()
    {
        return _joystick.getPOV();
    }

    public boolean getJoystickButtonValue(int buttonID)
    {
        return _joystick.getRawButton(buttonID);
	}
	
    public double getJoystickThrottle()
    {
        return _joystick.getThrottle();
	}
	
    public boolean isPovDirectionPressed(int povDirection)
	{
		double povValue = getPovAngle();
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
}
