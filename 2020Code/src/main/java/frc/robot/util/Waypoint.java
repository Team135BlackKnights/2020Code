/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class Waypoint
    {
        public double waypointX, waypointY, waypointTheta, waypointSpeed;

        public Waypoint(double x, double y, double theta, double speed)
        {

            waypointX = x;
            waypointY = y;
            waypointTheta = theta;
            waypointSpeed = speed;
        }
    }
