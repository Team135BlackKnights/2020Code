/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface RobotMap 
{

  public interface KOI 
  {
    public static final int 
      LEFT_JOYSTICK = 0,
      RIGHT_JOYSTICK = 1,
      MANIP_JOYSTICK = 2,

      TRIGGER_BUTTON = 1,
      THUMB_BUTTON = 2,

      HANDLE_BOTTOM_LEFT_BUTTON = 3,
      HANDLE_BOTTOM_RIGHT_BUTTON = 4, 
      HANDLE_TOP_LEFT_BUTTON = 5,
      HANDLE_TOP_RIGHT_BUTTON = 6,

      BASE_TOP_LEFT_BUTTON = 7,
      BASE_TOP_RIGHT_BUTTON = 8,
      BASE_MIDDLE_LEFT_BUTTON = 9,
      BASE_MIDDLE_RIGHT_BUTTON = 10,
      BASE_BOTTOM_LEFT_BUTTON = 11,
      BASE_BOTTOM_RIGHT_BUTTON = 12;

    public static final double 
      JOYSTICK_DEADBAND = .2;
      
  }
  public interface MOTORS 
  {
    
  final int FRONT_LEFT_SPARK_ID = 12,    FRONT_RIGHT_SPARK_ID = 13,
              REAR_LEFT_SPARK_ID = 11,     REAR_RIGHT_SPARK_ID  = 14;
    
  final int SPINNER_TALON_ID = 6,
              TURRET_TALON_ID = 4;

  final int BUTTON_CONTROL_TWO_TALON = 21, BUTTON_CONTROL_ONE_TALON = 22,
              MANIP_CONTROL_ONE_TALON = 23, MANIP_CONTROL_TWO_TALON = 24;

  final int FRONT_LEFT_FALCON = 1, FRONT_RIGHT_FALCON = 4,
            REAR_LEFT_FALCON = 2,  REAR_RIGHT_FALCON = 3; 
  }
  public interface CONSTANTS 
  {
  final int 
      ENCODER_TICKS_PER_REVOLUTION = 4096,
      WHEEL_DIAMETER = 6;

  }
  public interface SENSORS 
  {
  final int RIGHT_SONAR_TRIG = 0, RIGHT_SONAR_ECHO = 1,
            LEFT_SONAR_TRIG = 2, LEFT_SONAR_ECHO = 3,

            FRONT_LIDAR_ID = 8, BACK_LIDAR_ID = 9,
            TURRET_LIDAR = 10,  SHOOTER_TRIP_ID = 11,
            INTAKE_TRIP_ID = 12, INTAKE_SONAR_TRIG = 13, 
            INTAKE_SONAR_ECHO = 14;
            public SerialPort.Port navXPort = SerialPort.Port.kUSB;

  }

}
