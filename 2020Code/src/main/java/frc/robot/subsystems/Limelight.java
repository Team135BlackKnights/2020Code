package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

//import frc.robot.commands.*;

public class Limelight extends Subsystem {
	private static Limelight instance;

	NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
	NetworkTable limelightTable = networkTableInstance.getTable("limelight");
	NetworkTableEntry validTargetEntry = limelightTable.getEntry("tv"),
	horizontalOffsetEntry = limelightTable.getEntry("tx"),
	verticalOffsetEntry = limelightTable.getEntry("ty"),
	targetAreaEntry = limelightTable.getEntry("ta"),
	targetSkewEntry = limelightTable.getEntry("tl"),
	ledModeEntry = limelightTable.getEntry("ledMode"),
	cameraModeEntry = limelightTable.getEntry("camMode"),
	limelightPipelineEntry = limelightTable.getEntry("pipeline");

	public static final int NUMBER_OF_LIMELIGHT_CHARACTERISTICS = 5,
	VALID_TARGET = 0,
	HORIZONTAL_OFFSET = 1,
	VERTICAL_OFFSET = 2,
	TARGET_AREA = 3,
	TARGET_SKEW = 4;
	public double[] limelightData = new double[NUMBER_OF_LIMELIGHT_CHARACTERISTICS];

  public static int 
  LED_ON = 0, 
	LED_OFF = 1,
  LED_BLINKING = 2,

  VISION_PROCESSOR = 0,
  DRIVER_CAMERA = 1, 

  TARGET_PIPELINE = 0, 
  BALL_PIPELINE = 1, 
  VISION_PIPELINE = 2;
	

	public double[] GetLimelightData() { // creating an array so we can get to any of the values that we need from network tables
		limelightData[VALID_TARGET] = validTargetEntry.getDouble(0.0);
		limelightData[HORIZONTAL_OFFSET] = horizontalOffsetEntry.getDouble(0.0);
		limelightData[VERTICAL_OFFSET] = verticalOffsetEntry.getDouble(0.0);
		limelightData[TARGET_AREA] = targetAreaEntry.getDouble(0.0);
		limelightData[TARGET_SKEW] = targetSkewEntry.getDouble(0.0);
		return limelightData;
	}

	public void SetLEDMode(int onOrOff) {
		ledModeEntry.setNumber(onOrOff);
	}

	public void SetCameraMode(int cameraMode) {
		cameraModeEntry.setNumber(cameraMode);
	}

	public void SetCameraPipeline(int pipeline) {
		limelightPipelineEntry.setNumber(pipeline);
	}
	public void initLimelight(int ledMode, int pipeline)
	{
		SetLEDMode(ledMode);
		SetCameraPipeline(pipeline);
	}

	@Override
	public void periodic() 	{}

	public static Limelight getInstance() {if (instance == null) {instance = new Limelight();}return instance;}

	public void initDefaultCommand() {}
}