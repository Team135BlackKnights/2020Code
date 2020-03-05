/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.revrobotics.CANEncoder;

public class MotorControl {
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
    public static void resetEncoder(CANEncoder encoder) {
        encoder.setPosition(0);
    }

    // get current encoder position based off an encoder (returned in ticks)
    public static double getEncoderPosition(CANEncoder encoder) {
        return encoder.getPosition();
    }

    // Find rotations of an encoder from ticks given by getEncoderPosition()
    public static double ticksToRotations(double ticks) {
        return ticks / 4096;
    }

    // Get current conveyor rotations using getEncoderPosition() and
    // ticksToRotations()
    public static double getMotorRotations(CANEncoder encoder) {
        return ticksToRotations(getEncoderPosition(encoder));
    }

    // Get Conveyor Velocity
    public static double getMotorVelocity(CANEncoder encoder) {
        return encoder.getVelocity();
    }
}
