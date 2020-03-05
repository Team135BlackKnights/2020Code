/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.PathFIndingFIles;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.nsubsystems.FalconDrive;
/**
 * Add your docs here.
 */
public class PathFollower 
{
    public Segment[] segments;
    public Segment currentSegment;
    public FalconDrive drive; 
    public Waypoint[] waypoints;

    public int currentSegmentNumber;

    public double leftOutput, rightOutput, robotXPos, robotYPos, robotTheta, robotLinearSpeed, robotAngularSpeed, robotLeftSpeed, robotRightSpeed,
        leftPrevError = 0,
        rightPrevError = 0;

    public double lP, lI, lD, rP, rI, rD;

    public boolean doneWithPath = false;

    public PathFollower(Waypoint[] _waypoints, FalconDrive _drive)
    {
        currentSegmentNumber = 0;

        drive = _drive;

        robotXPos = drive.robotXPos();
        robotYPos = drive.robotYPos();
        robotTheta = drive.robotTheta();
        robotLinearSpeed = drive.getLinearMps();
        robotAngularSpeed = drive.getAngularMps();
        robotLeftSpeed = drive.getLeftMps();
        robotRightSpeed = drive.getRightMps();
        
        waypoints = _waypoints;
        segmentCreator(waypoints);
        currentSegment = segments[currentSegmentNumber];

        

    }

    public void followPath()
    {
       outputWaypointVals();
       updateRobotVals();
       segmentTransistion();
       SmartDashboard.putNumber("current Segment", currentSegmentNumber);
       checkForDone();
       if(currentSegment.isSegmentLine)
       {
        currentSegment.updateLinearSpeeds();
       }
       else 
       {
        currentSegment.updateCurvaLinearSpeeds();
       }
    

        lP = 1;
        lI = 0;
        lD = 0;

        rP = 1;
        rI = 0;
        rD = 0;

        double leftIntegral, leftDerivative, rightIntegral, rightDerivative, leftErrorSum, rightErrorSum, loopTime, leftError, rightError, leftDesired, rightDesired, maxVel;

        loopTime = .02;

        maxVel = 1.72; // Max velocity in M/s in low gear

        leftDesired = currentSegment.leftDesired ;
        rightDesired = currentSegment.rightDesired ;

        leftError = leftDesired * maxVel;
        rightError = rightDesired * maxVel; 

        leftErrorSum =+ leftError;
        rightErrorSum =+ rightError;

        leftIntegral = leftErrorSum * loopTime;
        rightIntegral = rightErrorSum * loopTime;

        leftDerivative = (leftError-leftPrevError)/loopTime;
        rightDerivative = (rightError - rightPrevError)/loopTime;
        double leftPrePID, rightPrePID;

        leftPrePID = leftDesired + leftError;
        rightPrePID = rightDesired + rightError;
        
        leftOutput = (leftPrePID *lP) + (leftIntegral *lI) + (leftDerivative *lD);
        rightOutput = (rightPrePID * rP) + (rightIntegral *lI) + (rightDerivative *lD);
        
        //drive.TankDrive(leftOutput, rightOutput);
        SmartDashboard.putNumber("Path follower left Error", leftError);
        SmartDashboard.putNumber("Path follower right error ", rightError);
        SmartDashboard.putNumber("path Follower Left Output", leftOutput);
        SmartDashboard.putNumber("Path follower Right Output ", rightOutput);

        leftPrevError = leftError;
        rightPrevError = rightError;
    }

    public void outputWaypointVals()
    {
        double 
        currentSegWaypointAX, currentSegWaypointAY, currentSegWaypointATheta, 
        currentSegWaypointBX, currentSegWaypointBY, currentSegWaypointBTheta,
        currentSegWaypointBS;

        currentSegWaypointATheta = currentSegment.A.waypointTheta;
        currentSegWaypointAX = currentSegment.A.waypointX;
        currentSegWaypointAY = currentSegment.A.waypointY;

        currentSegWaypointBTheta = currentSegment.B.waypointTheta;
        currentSegWaypointBX = currentSegment.B.waypointX;
        currentSegWaypointBY = currentSegment.B.waypointY;
        currentSegWaypointBS = currentSegment.B.waypointSpeed;

        SmartDashboard.putNumber("Waypoint A Theta", currentSegWaypointATheta);
        SmartDashboard.putNumber("Waypoint A X", currentSegWaypointAX);
        SmartDashboard.putNumber("Waypoint A Y ", currentSegWaypointAY);

        SmartDashboard.putNumber("Waypoint B Theta", currentSegWaypointBTheta);
        SmartDashboard.putNumber("Waypoint B X ", currentSegWaypointBX);
        SmartDashboard.putNumber("Waypoint B Y ", currentSegWaypointBY);

        SmartDashboard.putNumber("Waypoint B Speed", currentSegWaypointBS);

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
            isSegmentLine = 0 ==(A.waypointTheta - B.waypointTheta);

        }

        public void updateLinearSpeeds()
        {
            leftDesired = linearOutputs()[0];
            rightDesired = linearOutputs()[1];
        }

        public void updateCurvaLinearSpeeds()
        {
            leftDesired = curvalinearOutputs()[0];
            rightDesired = curvalinearOutputs()[1];
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
        double transistionDeadband = .05;
        
        currentSegment = segments[currentSegmentNumber];
        boolean isXNearEnough, isYNearEnough, isThetaNearEnough;
        isXNearEnough = isPointWithinDeadBand(robotXPos, currentSegment.B.waypointX, transistionDeadband);
        isYNearEnough = isPointWithinDeadBand(robotYPos, currentSegment.B.waypointY, transistionDeadband);
        isThetaNearEnough = isPointWithinDeadBand(robotTheta, currentSegment.B.waypointTheta, 5);
        
        if
        (isXNearEnough && isYNearEnough && isThetaNearEnough && currentSegmentNumber<n-1)/* || 
        (isXNearEnough && isThetaNearEnough && currentSegmentNumber<n) || 
        (isYNearEnough && isThetaNearEnough && currentSegmentNumber<n))
        */
        {
            currentSegmentNumber++;
           
        }
        
           
    }

    public boolean isPointEqual(double a, double b)
    {
        return a==b;
    }
    public boolean isPointWithinDeadBand(double a, double b, double deadband)
    {
        return Math.abs(b)-Math.abs(a) <=deadband;
    }

    public void checkForDone()
    {
        Segment finSegment = segments[segments.length-1];

       boolean isXNearEnough = isPointEqual(robotXPos, finSegment.B.waypointX);
       boolean isYNearEnough = isPointEqual(robotYPos, finSegment.B.waypointY);
       boolean isThetaTrue = isPointEqual(robotTheta, finSegment.B.waypointTheta);
       //boolean isSpeedTrue = isPointEqual(robotLinearSpeed, finSegment.B.waypointSpeed);

       if(isXNearEnough && isYNearEnough && isThetaTrue) //|| isSpeedTrue)
       {
        doneWithPath = true;
       }
    }

    double prevTheta = 0;
    public double[] curvalinearOutputs() 
    {
    Waypoint A,B;
    A = currentSegment.A;
    B = currentSegment.B;
    double pointA[] = {A.waypointX, A.waypointY};
    double pointB[] = {B.waypointX, B.waypointY};
    double rRadius = KnightMath.radiusFromPoints(pointA, pointB) - .2667;
    double lRadius = KnightMath.radiusFromPoints(pointA, pointB) + .2667;
    double dThetaL, dThetaR;
    double thetaL, thetaR;
    double chordL, chordR;
    double rVelocity, lVelocity;

    chordL = KnightMath.distanceFormula(pointA, pointB);
    thetaL = 2 * Math.asin(chordL/(2 * lRadius));
    dThetaL = (thetaL - prevTheta)/.02;
        lVelocity = lRadius * dThetaL;
    
    chordR = KnightMath.distanceFormula(pointA, pointB);
    thetaR= 2 * Math.asin(chordR/(2 * rRadius));
    dThetaR = (thetaR - prevTheta)/.02;
    rVelocity = rRadius * dThetaR;

    double velocities[] = {lVelocity, rVelocity};
    
    return velocities;
    }

    public double[] linearOutputs()
    {
        double a,b,c;

        Waypoint A, B;
        A = currentSegment.A;
        B = currentSegment.B;

        double pointA[] = {A.waypointX, A.waypointY};
        double pointB[] = {B.waypointX, B.waypointY};
        double pointR[] = {robotXPos, robotYPos};

        a = KnightMath.distanceFormula(pointB, pointR);
        b = KnightMath.distanceFormula(pointR, pointA);
        c = KnightMath.distanceFormula(pointA, pointB);

        double alpha = (Math.pow(c,2) + Math.pow(b,2) - Math.pow(a,2))/2*c*b;
        double pError = b*Math.sin(alpha);
        double kA = .24;
        double kS = 1; 
        double aError = B.waypointTheta-(robotTheta-(pError*kA));
        aError = aError * kS;
        double speed = B.waypointSpeed;

        double[] outputs = 
        {
            speed+aError,
            speed-aError
        };
        return outputs;
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