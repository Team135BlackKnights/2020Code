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
public interface RobotMap {

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
    // The convention for assigning IDS to motors will be based on which order is loosely based on its PDP power slot in reverse order 
    // ie PDP slot 15 goes to motor number 2 
    // The other part of the ID assignement is the motor controller type 
      // talon FX's will have a 0 or no number in front
      // Can Sparks will have a 1 in front of the id number
      // Talon SRX will have a 2 in front of the id number 
      // Victor SPX will have a 3 in front of the id number 


    final int FRONT_LEFT_SPARK_ID = 12,    FRONT_RIGHT_SPARK_ID = 13,
              REAR_LEFT_SPARK_ID = 11,     REAR_RIGHT_SPARK_ID  = 14;
    
    final int SPINNER_TALON_ID = 21;
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
