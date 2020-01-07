package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GetLimelightData extends Command {

	//  Creates an Array to Store the Data from the Limelight
	private double[] limelightData = new double[Limelight.NUMBER_OF_LIMELIGHT_CHARACTERISTICS];
	
    public GetLimelightData()
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.limelight);
    }

    // Called just before this Command runs the first time
    protected void initialize()
    {
    	Robot.limelight.SetCameraPipeline(Limelight.TARGET_PIPELINE);
    	Robot.limelight.SetCameraMode(Limelight.VISION_PROCESSOR);
    	Robot.limelight.SetLEDMode(Limelight.LED_OFF);  //  Turns off LED to Track the Yellow Block
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	limelightData = Robot.limelight.GetLimelightData();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished(){return false;}

    protected void end(){}
    protected void interrupted(){}
}
