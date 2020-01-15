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
    // The other part of the ID assignement is the motor controller type 
      // talon FX's will have a 0 or no number in front
      // Can Sparks will have a 1 in front of the id number
      // Talon SRX will have a 2 in front of the id number 
      // Victor SPX will have a 3 in front of the id number 


  final int FRONT_LEFT_SPARK_ID = 12,    FRONT_RIGHT_SPARK_ID = 13,
              REAR_LEFT_SPARK_ID = 11,     REAR_RIGHT_SPARK_ID  = 14;
    
  final int SPINNER_TALON_ID = 6,
              TURRET_TALON_ID = 4;

  final int BUTTON_CONTROL_TWO_TALON = 21, BUTTON_CONTROL_ONE_TALON = 22,
              MANIP_CONTROL_ONE_TALON = 23, MANIP_CONTROL_TWO_TALON = 24;

  final int FRONT_LEFT_FALCON = 1, FRONT_RIGHT_FALCON = 4,
            REAR_LEFT_FALCON = 2,  REAR_RIGHT_FALCON = 3; 
  }
  public interface SENSORS {
  final int RIGHT_SONAR_TRIG = 0, RIGHT_SONAR_ECHO = 1,
            LEFT_SONAR_TRIG = 2, LEFT_SONAR_ECHO = 3,

            FRONT_LIDAR_ID = 8, BACK_LIDAR_ID = 9,
            TURRET_LIDAR = 10,  SHOOTER_TRIP_ID = 11,
            INTAKE_TRIP_ID = 12, INTAKE_SONAR_TRIG = 13, 
            INTAKE_SONAR_ECHO = 14;

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
