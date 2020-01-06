
package frc.robot;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public interface MOTORS {

    final int
    FRONT_LEFT_SPARK_ID = 7, 
		FRONT_RIGHT_SPARK_ID = 1, 
		REAR_LEFT_SPARK_ID = 2,
		REAR_RIGHT_SPARK_ID = 3;

    final int colorSpinner = 1;
  }
  
}
