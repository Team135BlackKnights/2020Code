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

    public int currentSegmentNumber = 0;

    public Segment currentSegment = segments[currentSegmentNumber];

    public double leftOutput, rightOutPut, robotXPos, robotYPos, robotTheta, robotLinearSpeed, robotAngularSpeed, robotLeftSpeed, robotRightSpeed,
        leftPrevError = 0,
        rightPrevError = 0;

    public double lP, lI, lD, rP, rI, rD;

    public boolean doneWithPath = false;

    public PathFollower(Waypoint[] _waypoints, FalconDrive _drive)
    {
        drive = _drive;
        waypoints = _waypoints;
        segmentCreator(waypoints);
        updateRobotVals();

    }

    public void followPath()
    {
        updateRobotVals();

        segmentTransistion();

        lP = 1;
        lI = 0;
        lD = 0;

        rP = 1;
        rI = 0;
        rD = 0;

        double leftIntegral, leftDerivative, rightIntegral, rightDerivative, leftErrorSum, rightErrorSum, loopTime, leftError, rightError;
        loopTime = .02;
        leftError = currentSegment.leftError;
        rightError = currentSegment.rightError;

        leftErrorSum =+ leftError;
        rightErrorSum =+ rightError;

        leftIntegral = leftErrorSum * loopTime;
        rightIntegral = rightErrorSum * loopTime;

        leftDerivative = (leftError-leftPrevError)/loopTime;
        rightDerivative = (rightError - rightPrevError)/loopTime;

        
        leftOutput = (currentSegment.rightDesired* lP) + (leftIntegral *lI) + (leftDerivative *lD);
        rightOutPut = (currentSegment.leftDesired * rP) + (rightIntegral *lI) + (rightDerivative *lD);
        
        drive.TankDrive(leftOutput, rightOutPut);

        leftPrevError = leftError;
        rightPrevError = rightError;
    }

    public class Segment
    {
        Waypoint A, B;
        boolean isSegmentLine;
        double desiredDriveSpeed, leftDesired, rightDesired, leftError, rightError;
        public Segment(Waypoint a, Waypoint b)
        {
            A = a;
            B= b; 
            isSegmentLine = 0 <=(A.waypointTheta - B.waypointTheta);

            if(isSegmentLine && objectChecker(B, waypoints))
            {
               //TODO do line math 

               System.out.print("Driving along Line path");
            }     
            else if(!isSegmentLine && objectChecker(B, waypoints))
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
        //TODO add a deadband for segment transistion
        boolean isXTrue, isYTrue, isZTrue;
        isXTrue = isPointEqual(robotXPos, currentSegment.B.waypointX);
        isYTrue = isPointEqual(robotYPos, currentSegment.B.waypointY);
        isZTrue = isPointEqual(robotTheta, currentSegment.B.waypointTheta);
        
        if
        ((isXTrue && isYTrue && currentSegmentNumber<=n) || 
        (isXTrue && isZTrue && currentSegmentNumber<=n) || 
        (isYTrue && isZTrue && currentSegmentNumber<=n))

        {
            currentSegmentNumber++;
        }
           
    }

    public boolean isPointEqual(double a, double b)
    {
        return a==b;
    }

    public void checkForDone()
    {
        Segment finSegment = segments[segments.length-1];

       boolean isXTrue = isPointEqual(robotXPos, finSegment.B.waypointX);
       boolean isYTrue = isPointEqual(robotYPos, finSegment.B.waypointY);
       boolean isThetaTrue = isPointEqual(robotTheta, finSegment.B.waypointTheta);
       boolean isSpeedTrue = isPointEqual(robotLinearSpeed, finSegment.B.waypointSpeed);
    }

    public void arcFinder(Waypoint a, Waypoint b) {
        Waypoint A,B;
    A = currentSegment.A;
    B = currentSegment.B;
    double pointA[] = {A.waypointX, A.waypointY};
    double pointB[] = {B.waypointX, B.waypointY};
    double radius = KnightMath.radiusFromPoints(pointA, pointB);
    double dTheta;
    double theta;
    double chord;
    double arcLen;
    double dArcLen;
    double rightSpeed;
    double leftSpeed;

    chord = KnightMath.distanceFormula(pointA, pointB);

    }

    public boolean objectChecker(Object input, Object[] inputArray)
    {
        int n = inputArray.length;
        boolean isEqual = false;
        for(int i = 0; (i<n && !isEqual); i++)
        {
            isEqual = (input == inputArray[i]);
        }

        return isEqual;
    }

    public void linearPath()
    {   
        Waypoint A, B;

        C = KnightMath.distanceFormula(A, B);
        alpha = (Math.pow(C, 2) + Math.pow(B, 2) - Math.pow(A, 2)) / 2 * C * B;
        e = B * Math.asin(alpha);
        angleError = theta - (robotTheta - e * k);
        
        L = speed + angleError;
        R = speed - angleError;

    }

    public double[] linearOutputs;
    {
        c = KnightMath.distanceFormula(A, B)
    }


    public boolean segmentChecker(Segment input, Segment[] inputArray)
    {
        int n = inputArray.length;
        boolean isEqual = false;
        for(int i = 0; (i<n && !isEqual); i++)
        {
            isEqual = (input == inputArray[i]);
        }

        return isEqual;
    }

    public void updateRobotVals()
    {
        robotXPos = drive.robotXPos();
        robotYPos = drive.robotYPos();
        robotTheta = drive.robotTheta();
        robotLinearSpeed = drive.getLinearMps();
        robotAngularSpeed = drive.getAngularMps();
        robotLeftSpeed = drive.getLeftMps();
        robotRightSpeed = drive.getRightMps();
    }
}




