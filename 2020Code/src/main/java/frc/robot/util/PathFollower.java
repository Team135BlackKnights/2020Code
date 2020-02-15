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

    public int currentSegmentNumber = 0;

    public double leftOutput, rightOutPut, robotXPos, robotYPos, robotTheta, robotLinearSpeed, robotAngularSpeed;

    public PathFollower(Waypoint[] _waypoints, FalconDrive _drive)
    {
        drive = _drive;
        waypoints = _waypoints;

        robotXPos = drive.robotXPos();
        robotYPos = drive.robotYPos();
        robotTheta = drive.robotTheta();
        robotLinearSpeed = drive.getLinearMps();
        robotAngularSpeed = drive.getAngularMps();
    }

    public class Waypoint
    {
        public double waypointX, waypointY, waypointTheta, waypointsSpeed;

        public Waypoint(double x, double y, double theta, double speed)
        {

            waypointX = x;
            waypointY = y;
            waypointTheta = theta;
            waypointsSpeed = speed;
        }
    }

    public class Segment
    {
        Waypoint A, B;
        boolean isSegmentLine;
        double desiredDriveSpeed, leftDesired, rightDesired;
        public Segment(Waypoint a, Waypoint b)
        {
            A = a;
            B= b; 
            isSegmentLine = 0 <=(A.waypointTheta - B.waypointTheta);

        

            if(isSegmentLine && waypointChecker(B, waypoints))
            {
               //TODO do line math 

               System.out.print("Driving along Line path");
            }     
            else if(!isSegmentLine && waypointChecker(B, waypoints))
            {
                //TODO do arc math 
                System.out.print("Driving along Arc path");

            } 
            else 
            {
                System.out.println("Done with Path or invalid input");
            }

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

    public void segmentTransistion()
    {
        int n = segments.length;
        
        currentSegment = segments[currentSegmentNumber];

        boolean isXTrue, isYTrue, isZTrue;
        isXTrue = isPointEqual(robotXPos, currentSegment.B.waypointX);
        isYTrue = isPointEqual(robotYPos, currentSegment.B.waypointY);
        isZTrue = isPointEqual(robotTheta, currentSegment.B.waypointTheta);
        
        if((isXTrue && isYTrue) || (isXTrue && isZTrue) || (isYTrue && isZTrue))
        {
            currentSegmentNumber++;
        }
        
    }

    public boolean isPointEqual(double a, double b)
    {
        return a==b;
    }


    public void followPath()
    {

        double lP, lI, lD, rP, rI, rD;

        
    }

    public boolean waypointChecker(Waypoint input, Waypoint[] inputArray)
    {
        int n = inputArray.length;
        boolean isEqual = false;
        for(int i = 0; (i<n && !isEqual); i++)
        {
            isEqual = (input == inputArray[i]);
        }

        return isEqual;
    }

    public boolean segmentChecker(Segment input, Segment] inputArray)
    {
        int n = inputArray.length;
        boolean isEqual = false;
        for(int i = 0; (i<n && !isEqual); i++)
        {
            isEqual = (input == inputArray[i]);
        }

        return isEqual;
    }



