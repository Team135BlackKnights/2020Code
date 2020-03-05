/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

//File to use methods in most subsystems/commands without writing them out in each of them

public class MotorControl {
    //Our defualt set up for sparks
    public static void initCANSparkMax(CANSparkMax spark, boolean isBraked, boolean isInverted, int currentLimit)
  {
    spark.restoreFactoryDefaults();
    spark.setInverted(isInverted);
    spark.enableVoltageCompensation(12);
    if(isBraked)
    {
      spark.setIdleMode(IdleMode.kBrake);
    }
    else 
    {
      spark.setIdleMode(IdleMode.kCoast);
    }
    spark.setSmartCurrentLimit(currentLimit);
  }


    // Makes sure that the value given is within the limit setting it to the upper
    // or lower if it is above or below
    public static double limit(double x, double upperLimit, double lowerLimit) {
        if (x >= upperLimit) {
            x = upperLimit;
        } else if (x <= lowerLimit) {
            x = lowerLimit;
        }
        return x;
    }

    // set the encoder's current value to 0 (does not move conveyor)
    public static void resetSparkEncoder(CANEncoder encoder) {
        encoder.setPosition(0);
    }

    // get current encoder position based off an encoder (returned in ticks)
    public static double getSparkEncoderPosition(CANEncoder encoder) {
        return encoder.getPosition();
    }

    // Find rotations of an encoder from ticks given by getEncoderPosition()
    public static double ticksToRotations(double ticks) {
        return ticks / 4096;
    }

    // Get current conveyor rotations using getEncoderPosition() and
    // ticksToRotations()
    public static double getSparkEncoderRotations(CANEncoder encoder) {
        return ticksToRotations(getSparkEncoderPosition(encoder));
    }

    // Get Conveyor Velocity
    public static double getSparkVelocity(CANEncoder encoder) {
        return encoder.getVelocity();
    }
}
