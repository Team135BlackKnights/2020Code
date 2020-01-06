/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public interface KOI {
    public static final int 
      LEFT_JOYSTICK = 0,
      RIGHT_JOYSTICK = 1,
      MANIP_JOYSTICK = 2,
      TRIGGER_BUTTON = 1;


    public static final double 
      JOYSTICK_DEADBAND = .2;
      
  }
  public interface MOTORS {

    final int FRONT_LEFT_SPARK_ID = 7, 
		FRONT_RIGHT_SPARK_ID = 1, 
		REAR_LEFT_SPARK_ID = 2,
		REAR_RIGHT_SPARK_ID = 3;
    final int colorSpinner = 1;
  }
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
