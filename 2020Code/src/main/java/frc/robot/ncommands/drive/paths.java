/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;

import frc.robot.util.Waypoint;

/**
 * Add your docs here.
 */
public class paths
{
    public static Waypoint[] redRightWaypoints()
    {
        Waypoint[] waypoints = 
        {
            new Waypoint(0, 0, 0, 0),
            new Waypoint(1, 0, 0, .5),
            new Waypoint(1, .5, 90, .5),
            new Waypoint(1, .5 , 90, 0)
        };

        return waypoints;
    }
    
}