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

    public double leftOutput, rightOutPut, robotXPos, robotYPos, endXPos, endYPos, robotTheta, robotLinearSpeed, robotAngularSpeed;

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

    public void linearPath()
    {       

    }

    public void linearCorrection()
    {
        double x1, x2, y1, y2, theta1, theta2, s1, s2, A, B, C, a, b, c;
        A = waypoint(x1, y1, theta1, s1);
        B = waypoint(x2, y2, theta2, s2);
        C = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
        
    }
}
