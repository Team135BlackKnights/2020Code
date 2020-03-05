/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.util.MotorControl;

public class Turret extends SubsystemBase implements RobotMap.TURRET{
  /**
   * Creates a new newTurret.
   */

  public CANSparkMax rotationSpark, hoodSpark, shooterMaster, shooterSlave, indexerSpark;
  public CANEncoder rotationEncoder, hoodEncoder, shooterEncoder, indexEncoder;
  public DigitalInput ballTrip; 
  public double maxRPM;   

  NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
	NetworkTable TurretLimelightTable = networkTableInstance.getTable("limelight-turret");
	NetworkTableEntry validTargetEntry = TurretLimelightTable.getEntry("tv"),
			horizontalOffsetEntry = TurretLimelightTable.getEntry("tx"),
			verticalOffsetEntry = TurretLimelightTable.getEntry("ty"),
			targetAreaEntry = TurretLimelightTable.getEntry("ta"),
			targetSkewEntry = TurretLimelightTable.getEntry("tl"),
			ledModeEntry = TurretLimelightTable.getEntry("ledMode"),
			cameraModeEntry = TurretLimelightTable.getEntry("camMode"),
      limelightPipelineEntry = TurretLimelightTable.getEntry("pipeline");
      
public double[] limelightData = new double[5];

public static final int 
  VALID_TARGET = 0, 
  HORIZONTAL_OFFSET = 1,
  VERTICAL_OFFSET = 2,
  TARGET_AREA = 3, 
  TARGET_SKEW = 4,

    LED_ON = 0, 
    LED_OFF = 1,
    LED_BLINKING = 2,

    VISION_PROCESSOR = 0,
    DRIVER_CAMERA = 1,

    REFLECTIVE_TAPE = 0,
    DRIVER_VISION = 1;

  public boolean currentState, previousState, isReadyForBall;
  public int ballsShot;

  public Turret() 
  {
    rotationSpark = new CANSparkMax(ROTATION_SPARK_ID, MotorType.kBrushless);
    hoodSpark = new CANSparkMax(TILT_SPARK_ID, MotorType.kBrushless);
    shooterMaster = new CANSparkMax(TOP_SHOOTER_SPARK_ID, MotorType.kBrushless);
    shooterSlave = new CANSparkMax(BOTTOM_SHOOTER_SPARK_ID, MotorType.kBrushless);
    indexerSpark = new CANSparkMax(FEEDER_SPARK_ID, MotorType.kBrushless);

    rotationEncoder = new CANEncoder(rotationSpark);
    hoodEncoder = new CANEncoder(hoodSpark);
    shooterEncoder = new CANEncoder(shooterMaster);
    indexEncoder = new CANEncoder(indexerSpark);

    ballTrip = new DigitalInput(15);

    MotorControl.initCANSparkMax(rotationSpark, true, false, 20);
    MotorControl.initCANSparkMax(hoodSpark, true, false, 20);
    MotorControl.initCANSparkMax(shooterMaster, true, false, 30);
    MotorControl.initCANSparkMax(shooterSlave, true, false, 30);
    MotorControl.initCANSparkMax(indexerSpark, true, true, 30);

    shooterSlave.follow(shooterMaster);

    currentState = ballTrip.get();
    previousState = false;
    isReadyForBall = false;
    ballsShot = 0; 
    maxRPM = 5100;

    initLimelight(LED_OFF, DRIVER_CAMERA);

    System.out.print("new Turret Initialized");
  }

  

  public void resetShooterEncoder()
  {
    shooterEncoder.setPosition(0);
  }

  public void resetRotationEncoder()
  {
    rotationEncoder.setPosition(0);
  }

  public void resetHoodEncoder()
  {
    hoodEncoder.setPosition(0);
  }
  public void resetIndexerEnocder()
  {
    indexEncoder.setPosition(0);
  }

  public void resetAllTurretEncoders()
  {
    resetShooterEncoder();
    resetRotationEncoder();
    resetHoodEncoder();
    resetIndexerEnocder();
  }

  public double getShooterEncoderPos()
  {
    return shooterEncoder.getPosition();
  }
  
  public double getRotationPos()
  {
    return rotationEncoder.getPosition();
  }

  public double getHoodPos()
  {
    return hoodEncoder.getPosition();
  }

  public double getIndexerPos()
  {
    return indexEncoder.getPosition();
  }

  public double getShooterVel()
  {
    return shooterEncoder.getVelocity();
  }

  public void runHood(double power)
  {
    power = limit(power, .45, -.45);
    hoodSpark.set(power);
  }

  public void runRotation(double power)
  {
    power = limit(power, .85 , -.85);
    rotationSpark.set(power);
  }

  public void runIndexer(double power)
  {
    power = limit(power, .85, -.85);
    indexerSpark.set(power);
  }

  public void runLimitedRotation(double power)
  {
    double reverseLimit = -250, forwardLimit = 35;
    if(getRotationPos() >=forwardLimit)
    {
      power = limit(power, 0, -.45);
    }
    else if (getRotationPos() <= reverseLimit)
    {
      power = limit(power, .45, 0);
    }
    else 
    {
      power = limit(power, .45, -.45);
    }
    rotationSpark.set(power);
  }

  public void runLimitedHood(double power)
  {
    double reverseLimit = 0, forwardLimit = 100;
    if(getHoodPos() >= forwardLimit)
    {
      power = limit(power, 0, -.45);
    }
    else if(getHoodPos() <=reverseLimit)
    {
      power = limit(power, .45, 0);
    }
    else 
    {
      power = limit(power, .45,-.45);
    }
    hoodSpark.set(power);
  }

  public void runShooter(double power)
  {
    power = limit(power, .9, -.9);
    shooterMaster.set(power);
  }

  public void aimTurret(double rotationPower, double hoodPower)
  {
    runLimitedHood(hoodPower);
    runLimitedRotation(rotationPower);
  }

  public boolean isBallInTurret()
  {
    return ballTrip.get();
  }
  

  public double limit(double x, double upperLimit, double lowerLimit) {
    if (x >= upperLimit) {
      x = upperLimit;
    } else if (x <= lowerLimit) {
      x = lowerLimit;
    }
    return x;
  }

  public void stopShooter()
  {
    shooterMaster.set(0);
  }
  
  public void stopTurret()
  {
    aimTurret(0, 0);
  }

  public void stopAllMotors()
  {
    stopShooter();
    stopTurret();
  }

  public void updateBallCount()
  {
    currentState = isBallInTurret();
    if(previousState && previousState !=currentState)
    {
      RobotContainer.activeBallCount--;
      ballsShot++;
    }
    previousState = currentState;
  }

  public void printBallData()
  {
    SmartDashboard.putBoolean("Is Ball in Shooter", isBallInTurret());
    SmartDashboard.putNumber("Current Balls in System", RobotContainer.activeBallCount);
    SmartDashboard.putNumber("Balls Shot", ballsShot);
  }

  public void printShooterData()
  {
    SmartDashboard.putNumber("Shooter RPM: ", getShooterVel());
    SmartDashboard.putNumber("Shooter Temp: ", shooterMaster.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Current: ", shooterMaster.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Output: ", shooterMaster.getAppliedOutput());
  }

  public void printRotationData()
  {
    SmartDashboard.putNumber("Rotation Pos: ", getRotationPos());
    SmartDashboard.putNumber("Rotation Temp: ", rotationSpark.getMotorTemperature());
    SmartDashboard.putNumber("Rotation Current: ", rotationSpark.getOutputCurrent());
    SmartDashboard.putNumber("Rotation Output: ", rotationSpark.getAppliedOutput());
  }

  public void printHoodData()
  {
    SmartDashboard.putNumber("Hood Pos: ", getHoodPos());
    SmartDashboard.putNumber("Hood Temp: ", hoodSpark.getMotorTemperature());
    SmartDashboard.putNumber("Hood Current: ", hoodSpark.getOutputCurrent());
    SmartDashboard.putNumber("Hood Output: ", hoodSpark.getAppliedOutput());
  }

  public void printIndexerData()
  {
    SmartDashboard.putNumber("Indexer Pos: ", getIndexerPos());
    SmartDashboard.putNumber("Indexer Temp: ", indexerSpark.getMotorTemperature());
    SmartDashboard.putNumber("Indexer Current: ", indexerSpark.getOutputCurrent());
    SmartDashboard.putNumber("Indexer Output: ", indexerSpark.getAppliedOutput());
  }

  public double[] getLimelightData() 
  {
    limelightData[VALID_TARGET] = validTargetEntry.getDouble(0);
    limelightData[HORIZONTAL_OFFSET] = horizontalOffsetEntry.getDouble(0);
    limelightData[VERTICAL_OFFSET] = verticalOffsetEntry.getDouble(0);
    limelightData[TARGET_AREA] = targetAreaEntry.getDouble(0);
    limelightData[TARGET_SKEW] = targetSkewEntry.getDouble(0);

    return limelightData;
  }

  public double distanceToTarget()
  {

    double targetHeight = 8;
    double limeligtHeight = 34/12;
    double h = targetHeight-limeligtHeight;

    double limelightAngle = 25;
    double targetAngle = getLimelightData()[VERTICAL_OFFSET];

    double theta = limelightAngle -targetAngle;
    return h/Math.tan(theta);
  }

  public void printLimelightData()
  {
    SmartDashboard.putNumber("Distance To Target", distanceToTarget());
    SmartDashboard.putBoolean("is Target Valid ?: ", limelightData[VALID_TARGET] >=1);
    SmartDashboard.putNumber("Horizontal Offset", limelightData[HORIZONTAL_OFFSET]);
    SmartDashboard.putNumber("Vertical Offset", limelightData[VERTICAL_OFFSET]);
    SmartDashboard.putNumber("Target Area", limelightData[TARGET_AREA]);
    SmartDashboard.putNumber("Target Skew", limelightData[TARGET_SKEW]);
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

	public void initLimelight(int ledMode, int pipeline) {
		SetLEDMode(ledMode);
		SetCameraPipeline(pipeline);
  }

  public void autoIndexBall()
  {
    if(isBallInTurret())
    {
      resetIndexerEnocder();
    }

    if(isReadyForBall && getIndexerPos() <= 5)
    {
      runIndexer(.5);
    }
  }

  
  
  @Override
  public void periodic() 
  {
    isBallInTurret();
    updateBallCount();
    autoIndexBall();
    // This method will be called once per scheduler run
  }
}
