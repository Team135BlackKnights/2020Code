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
public class checker 
{
    public static boolean objectChecker(Object input, Object[] inputArray)
    {
        int n = inputArray.length;
        boolean isEqual = false;
        for(int i = 0; (i<n && !isEqual); i++)
        {
            isEqual = (input == inputArray[i]);
        }

        return isEqual;
    }
}
