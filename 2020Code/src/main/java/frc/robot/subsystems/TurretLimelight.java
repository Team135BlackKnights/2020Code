package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

//import frc.robot.commands.*;

public class TurretLimelight extends Subsystem {
	private static TurretLimelight instance;
	
	NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
	NetworkTable TurretLimelightTable = networkTableInstance.getTable("limelight-turret");
	NetworkTableEntry 
	validTargetEntry = TurretLimelightTable.getEntry("tv"),
	horizontalOffsetEntry = TurretLimelightTable.getEntry("tx"),
	verticalOffsetEntry = TurretLimelightTable.getEntry("ty"),
	targetAreaEntry = TurretLimelightTable.getEntry("ta"),
	targetSkewEntry = TurretLimelightTable.getEntry("tl"),
	ledModeEntry = TurretLimelightTable.getEntry("ledMode"),
	cameraModeEntry = TurretLimelightTable.getEntry("camMode"),
	limelightPipelineEntry = TurretLimelightTable.getEntry("pipeline");

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
	public void periodic() 	
	{
		SmartDashboard.putNumber(" Horizontal offset", GetLimelightData()[HORIZONTAL_OFFSET]);
		SmartDashboard.putNumber(" Vertical Offset", limelightData[VERTICAL_OFFSET]);
		SmartDashboard.putBoolean("Target Exist", limelightData[0] >=1);
	}

	public static TurretLimelight getInstance() {if (instance == null) {instance = new TurretLimelight();}return instance;}

	public void initDefaultCommand() {}
}