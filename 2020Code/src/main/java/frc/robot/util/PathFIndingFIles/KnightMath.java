/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.PathFIndingFIles;


public class KnightMath 
{
    public static double[] centroid(double[] point1, double[] point2 )
    {
        double x1, y1, x2, y2;
        x1 = point1[0];
        y1 = point1[1];

        x2 = point2[0];
        y2 = point2[1];


        double[] centroidPoints = new double[2];

        centroidPoints[0] = (2*x1 + x2)/3;
        centroidPoints[1] = (y1 + 2*y2)/3;
        
        return centroidPoints;
    }

    public static double radiusFromPoints(double[] point1, double[] point2)
    {
        double x1, y1, x2, y2, x3, y3;

        double[] point3 = centroid(point1, point2);

        x1 = point1[0];
        y1 = point1[1];
        x2 = point2[0];
        y2 = point2[1];
        x3 = point3[0];
        y3 = point3[1];

        double x12, x13, y12, y13, x31, x21, y31, y21;

        x12 = x1 - x2; 
        x13 = x1 - x3; 
    
        y12 = y1 - y2; 
        y13 = y1 - y3; 
    
        y31 = y3 - y1; 
        y21 = y2 - y1; 
    
        x31 = x3 - x1; 
        x21 = x2 - x1; 
        
        double sx13, sy13, sx21, sy21;
        // x1^2 - x3^2 
        sx13 = (int)(Math.pow(x1, 2) - 
                        Math.pow(x3, 2)); 
    
        // y1^2 - y3^2 
        sy13 = (int)(Math.pow(y1, 2) - 
                        Math.pow(y3, 2)); 
    
        sx21 = (int)(Math.pow(x2, 2) - 
                        Math.pow(x1, 2)); 
                        
        sy21 = (int)(Math.pow(y2, 2) - 
                        Math.pow(y1, 2)); 

        double f, g, c, h, k, sqrR;

        f = ((sx13) * (x12) 
            + (sy13) * (x12) 
            + (sx21) * (x13) 
            + (sy21) * (x13)) 
            / (2 * ((y31) * (x12) - (y21) * (x13))); 
        g = ((sx13) * (y12) 
            + (sy13) * (y12) 
            + (sx21) * (y13) 
            + (sy21) * (y13)) 
            / (2 * ((x31) * (y12) - (x21) * (y13))); 
  
        c = -(int)Math.pow(x1, 2) - (int)Math.pow(y1, 2) - //Center point of circle
                                2 * g * x1 - 2 * f * y1; 
  
    // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0 
    // where centre is (h = -g, k = -f) and radius r 
    // as r^2 = h^2 + k^2 - c 
        h = -g; 
        k = -f; 
        sqrR = h * h + k * k - c; 
        

        double radius = Math.sqrt(sqrR);
        
        return radius;
    }

    public static double distanceFormula(double[] A, double[] B)
    {
        double x1, y1, x2, y2, C;
        x1 = A[0];
        y1 = A[1];
        x2 = B[0];
        y2 = B[1];
        C = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
        return C; 
    }
}