/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class IntakeLimelight extends SubsystemBase {
  /**
   * Creates a new IntakeLimelight.
   */
  private static IntakeLimelight instance;
	
	NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
	NetworkTable intakeLimelightTable = networkTableInstance.getTable("limelight-intake");
	NetworkTableEntry 
	validTargetEntry = intakeLimelightTable.getEntry("tv"),
	horizontalOffsetEntry = intakeLimelightTable.getEntry("tx"),
	verticalOffsetEntry = intakeLimelightTable.getEntry("ty"),
	targetAreaEntry = intakeLimelightTable.getEntry("ta"),
	targetSkewEntry = intakeLimelightTable.getEntry("tl"),
	ledModeEntry = intakeLimelightTable.getEntry("ledMode"),
	cameraModeEntry = intakeLimelightTable.getEntry("camMode"),
	limelightPipelineEntry = intakeLimelightTable.getEntry("pipeline");

	// Lists different situations with target
	public static final int
	VALID_TARGET = 0,
	HORIZONTAL_OFFSET = 1,
	VERTICAL_OFFSET = 2,
	TARGET_AREA = 3,
	TARGET_SKEW = 4;
	public double[] limelightData = new double[5];

	public static int 
	// LED congigs
    LED_ON = 0, 
	LED_OFF = 1,
    LED_BLINKING = 2,

	// Camera Configs
    VISION_PROCESSOR = 0,
    DRIVER_CAMERA = 1, 

	// Pipeline Configs
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

	public void SetLEDMode(int onOrOff) { // LED Configs
		ledModeEntry.setNumber(onOrOff);
	}

	public void SetCameraMode(int cameraMode) { // Camera Configs
		cameraModeEntry.setNumber(cameraMode);
	}

	public void SetCameraPipeline(int pipeline) { // Pipeline Configs
		limelightPipelineEntry.setNumber(pipeline);
	}
	public void initLimelight(int ledMode, int pipeline)
	{
		SetLEDMode(ledMode);
		SetCameraPipeline(pipeline);
	}

	
	@Override
	public void periodic() 	
	{
	//	SmartDashboard.putNumber(" Horizontal offset", GetLimelightData()[HORIZONTAL_OFFSET]);
	//	SmartDashboard.putNumber(" Vertical Offset", limelightData[VERTICAL_OFFSET]);
	//	SmartDashboard.putBoolean("Target Exist", limelightData[0] >=1);
	}

	public static IntakeLimelight getInstance() {if (instance == null) {instance = new IntakeLimelight();}return instance;}

}
