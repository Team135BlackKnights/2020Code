/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.nsubsystems.FalconDrive;

/**
 * Add your docs here.
 */
public class PathFollower 
{
    public Segment[] segments;
    public FalconDrive drive; 
    public Waypoint[] waypoints;

    public Segment currentSegment;

    public PathFollower(Waypoint[] _waypoints, FalconDrive _drive)
    {
        drive = _drive;
        waypoints = _waypoints;
    }

    public class Waypoint
    {
        public Waypoint(double x, double y, double theta, double speed)
        {
            double waypointX, waypointY, waypointTheta, waypointsSpeed;

            waypointX = x;
            waypointY = y;
            waypointTheta = theta;
            waypointsSpeed = speed;
        }
    }

    public class Segment
    {
        public Segment(Waypoint a, Waypoint b)
        {
            Waypoint aCopy = a;
            Waypoint bCopy = b; 

            //boolean isSegmentLine = 0 <=(aCopy.waypointTheta - bCopy.waypointTheta);
        }

        
    }

    public void segmentCreator(Waypoint[] waypoints)
    {
        int n = waypoints.length;
        segments = new Segment[n-1];

        for(int i = 0; i<n-1; i++)
        {
            segments[i] = new Segment(waypoints[i], waypoints[i+1]);
        }
    }


    public void followPath()
    {

    }
}
