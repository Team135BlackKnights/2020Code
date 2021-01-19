/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface RobotMap {

  public interface KOI {
    // Variables for joysticks and their buttons initiation
    public static final int LEFT_JOYSTICK = 0, RIGHT_JOYSTICK = 2, MANIP_JOYSTICK = 1,

        TRIGGER_BUTTON = 1, THUMB_BUTTON = 2,

        HANDLE_BOTTOM_LEFT_BUTTON = 3, HANDLE_BOTTOM_RIGHT_BUTTON = 4, HANDLE_TOP_LEFT_BUTTON = 5,
        HANDLE_TOP_RIGHT_BUTTON = 6,

        BASE_TOP_LEFT_BUTTON = 7, BASE_TOP_RIGHT_BUTTON = 8, BASE_MIDDLE_LEFT_BUTTON = 9, BASE_MIDDLE_RIGHT_BUTTON = 10,
        BASE_BOTTOM_LEFT_BUTTON = 11, BASE_BOTTOM_RIGHT_BUTTON = 12;

    public static final double JOYSTICK_DEADBAND = .2;
  }

  // Variables used for the drivetrain initiation
  public interface DRIVE {
    final int FRONT_LEFT_FALCON = 3, FRONT_RIGHT_FALCON = 1, REAR_LEFT_FALCON = 4, REAR_RIGHT_FALCON = 2,

        SHIFTER_ID = 1,

        WHEEL_DIAMETER = 6, ENCODER_TICKS_PER_REVOLUTION = 4096,

        ksVolts = 12, kvVoltSecondsPerMeter = 1, kaVoltSecondsSquaredPerMeter = 1;

    public SerialPort.Port navXPort = SerialPort.Port.kUSB;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(.7);

  }

  // Variables for Turret subsystem initiation
  public interface TURRET {
    public static final int ROTATION_SPARK_ID = 11, SLAVE_SHOOTER_SPARK_ID = 12, MASTER_SHOOTER_SPARK_ID = 13,
        FEEDER_SPARK_ID = 14, TILT_SPARK_ID = 20, LEFT_LIMIT_ID = 1, RIGHT_LIMIT_ID = 2, TILT_LIMIT_ID = 3,
        TARGETING_LIGHT = 0;

  }

  // Variables for control over the color wheel
  public interface CONTROL_PANEL {
    public static final int ROTATOR_ID = 16;

    // Blue color mins and maxes
    public double BlueRedMin = .09;
    public double BlueRedMax = .21;
    public double BlueGreenMin = .40;
    public double BlueGreenMax = .50;
    public double BlueBlueMin = .33;
    public double BlueBlueMax = .48;

    // Green color mins and maxes
    public double GreenRedMin = .15;
    public double GreenRedMax = .2;
    public double GreenGreenMin = .5;
    public double GreenGreenMax = .59;
    public double GreenBlueMin = .24;
    public double GreenBlueMax = .27;

    // Red color mins and maxes
    public double RedRedMin = .38;
    public double RedRedMax = .58;
    public double RedGreenMin = .31;
    public double RedGreenMax = .40;
    public double RedBlueMin = .09;
    public double RedBlueMax = .18;

    // Yellow color mins and maxes
    public double YellowRedMin = .29;
    public double YellowRedMax = .35;
    public double YellowGreenMin = .49;
    public double YellowGreenMax = .58;
    public double YellowBlueMin = .10;
    public double YellowBlueMax = .17;

  }

  // Variables for Endgame initiation
  public interface ENDGAME {
    public static final int WIND_UP_SPARK_ID = 15, LIFT_UP_SPARK_ID = 17;

  }

  // Variables for Intake inititation
  public interface INTAKE {
    final int CONVEYOR_SPARK = 18, ROLLER_SPARK = 19,

        RAISE_LOWER = 0,

        INTAKE_TRIP_SWITCH = 0;
  }

}