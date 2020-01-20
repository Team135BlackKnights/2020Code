/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.drive.*;

/**
 * Add your docs here.
 */
public class FalconDrive extends Subsystem implements RobotMap{

  //
  public static FalconDrive instance; 
  //Declares four Falcon 500 Motors
  public WPI_TalonFX frontLeftFX, frontRightFX, rearLeftFX, rearRightFX;

  public SpeedControllerGroup leftDriveSide, rightDriveSide; 
  public DifferentialDrive chassis; 

  public Ultrasonic rightSonar,leftSonar;
  public  Counter testLidar; 
  public AHRS navx;

    public static FalconDrive getInstance()
  {
     if (instance == null) {
        instance = new FalconDrive();
     } return instance;
     
  }

  public FalconDrive()
  {
    frontLeftFX = new WPI_TalonFX(MOTORS.FRONT_LEFT_FALCON);
    rearLeftFX = new WPI_TalonFX(MOTORS.REAR_LEFT_FALCON);

    frontRightFX = new WPI_TalonFX(MOTORS.FRONT_RIGHT_FALCON);
    rearRightFX = new WPI_TalonFX(MOTORS.REAR_RIGHT_FALCON);
    
    configFalcon(frontLeftFX, true);
    configFalcon(rearLeftFX, true);
    configFalcon(frontRightFX, true);
    configFalcon(rearRightFX, true);

    leftSonar = new Ultrasonic(SENSORS.LEFT_SONAR_TRIG, SENSORS.LEFT_SONAR_ECHO);
    rightSonar = new Ultrasonic(SENSORS.RIGHT_SONAR_TRIG, SENSORS.RIGHT_SONAR_ECHO);

    rightSonar.setAutomaticMode(true);
    leftSonar.setAutomaticMode(true);

    testLidar = new Counter(SENSORS.FRONT_LIDAR_ID);
    initLidar(testLidar); 

    navx = new AHRS(SENSORS.navXPort);
    navx.reset();

    leftDriveSide = new SpeedControllerGroup(frontLeftFX, rearLeftFX);
    rightDriveSide = new SpeedControllerGroup(frontRightFX, rearRightFX);
  
    chassis = new DifferentialDrive(leftDriveSide, rightDriveSide);

    chassis.setSafetyEnabled(false);
    chassis.setMaxOutput(.98);
    
    resetEncoders();
    setBrakeMode(NeutralMode.Brake);
    
    System.out.println("Falcon Initialized");

  }

  public void configFalcon(WPI_TalonFX falcon, boolean isLeft)
  { 
    falcon.setNeutralMode(NeutralMode.Brake);
    falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,100);
    falcon.configVoltageCompSaturation(12.0, 100);
    falcon.enableVoltageCompensation(true);
    falcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,5, 100);
    falcon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms,100);
    falcon.configVelocityMeasurementWindow(1,100);    
    //falcon.setSensorPhase(isLeft);
  }

  public void setBrakeMode(NeutralMode neutralMode)
  {
    frontLeftFX.setNeutralMode(neutralMode);
    frontRightFX.setNeutralMode(neutralMode);
    rearLeftFX.setNeutralMode(neutralMode);
    rearRightFX.setNeutralMode(neutralMode);

  }

  public void TankDrive(double leftPower, double rightPower)
  {
    chassis.tankDrive(leftPower, rightPower);
  }

  public void ArcadeDrive(double lateralPower, double rotationalPower)
  { 
    chassis.arcadeDrive(lateralPower, -rotationalPower);
  }
  public void CurvatureDrive(double xSpeed, double zRotation)
  {
     chassis.curvatureDrive(xSpeed, zRotation, true);
  }
 
  public void resetEncoders()
  {
    frontLeftFX.setSelectedSensorPosition(0);
    frontRightFX.setSelectedSensorPosition(0);
    rearLeftFX.setSelectedSensorPosition(0);
    rearRightFX.setSelectedSensorPosition(0);
    SmartDashboard.putBoolean("Encoders Reset:", true);
  }

  public double getEncoderDistance(TalonFX falcon)
  {
    return falcon.getSelectedSensorPosition()/4096;
  }

  public double getEncoderVelocity(TalonFX falcon)
  {
    return falcon.getSelectedSensorVelocity();
  }

 
  public void initLidar(Counter lidar)
  {
    lidar.setMaxPeriod(1.00);
    lidar.setSemiPeriodMode(true);
    lidar.reset();
  }

  public double lidarDistance(Counter lidar)
  {
    double lidarDistance;
    if(lidar.get() < 1)
      lidarDistance = 0;
    else 
      lidarDistance = (lidar.getPeriod() * 1000000.0 / 10.0);
    return lidarDistance;
  }

  public double getAngle()
  {
   SmartDashboard.putNumber("Current angle of Robot:", navx.getYaw());
   return navx.getYaw();
 }
 public double getRotationRate()
 {
   return navx.getRate();
 }

  public void resetGyro()
  {
    navx.zeroYaw();
  }

    public double limit(double x, double upperLimit, double lowerLimit)
	{	if(x >= upperLimit){ x = upperLimit;}
		else if( x<=lowerLimit){ x = lowerLimit;}
		return x;
	}

     public double getLeftPos()
  {
    return (getEncoderDistance(frontLeftFX) + getEncoderDistance(rearLeftFX))/2;
  }
  public double getRightPos()
  {
    return (getEncoderDistance(frontRightFX) + getEncoderDistance(rearRightFX))/2;
  }

    public void printPositions()
  {
    SmartDashboard.putNumber("front Left Position:", getEncoderDistance(frontLeftFX));
    SmartDashboard.putNumber("rear Left Position:", getEncoderDistance(rearLeftFX));
    SmartDashboard.putNumber("front Right Position:", getEncoderDistance(frontRightFX));
    SmartDashboard.putNumber("rear Right Position:", getEncoderDistance(rearRightFX));
    
    SmartDashboard.putNumber("Average Left Position:", getLeftPos());
    SmartDashboard.putNumber("Average Right Position:", getRightPos());

    
  } 
  
  public double getMotorPercent(WPI_TalonFX falcon)
  {
    return falcon.getMotorOutputPercent();
  }
  public double getMotorVoltage(WPI_TalonFX falcon)
  {
    return falcon.getMotorOutputVoltage();
  }
  public double getMotorTemp(WPI_TalonFX falcon)
 {
  return falcon.getTemperature();
 }
 public void printPower()
  {
    SmartDashboard.putNumber("front Left Output Percent: ", getMotorPercent(frontLeftFX));
    SmartDashboard.putNumber("rear Left Output Percent: ", getMotorPercent(rearLeftFX));
    SmartDashboard.putNumber("front Right Output Percent: ", getMotorPercent(frontRightFX));
    SmartDashboard.putNumber("rear Right Output Percent: ", getMotorPercent(rearRightFX));
  }
  public void printVoltage()
  {
    SmartDashboard.putNumber("front Left Voltage: ", getMotorVoltage(frontLeftFX));
    SmartDashboard.putNumber("rear Left Voltage: ", getMotorVoltage(rearLeftFX));
    SmartDashboard.putNumber("front Right Voltage: ", getMotorVoltage(frontRightFX));
    SmartDashboard.putNumber("rear Right Voltage: ", getMotorVoltage(rearRightFX));
  }
  public void printTemperature()
  {
    SmartDashboard.putNumber("front Left Temp: ", getMotorVoltage(frontLeftFX));
    SmartDashboard.putNumber("rear Left Voltage: ", getMotorVoltage(rearLeftFX));
    SmartDashboard.putNumber("front Right Voltage: ", getMotorVoltage(frontRightFX));
    SmartDashboard.putNumber("rear Right Voltage: ", getMotorVoltage(rearRightFX));
  }
  
  public double sonarDistance(Ultrasonic sonar)
  {
    return sonar.getRangeInches();
  }

  public void printUltrasonicValues() {
    SmartDashboard.putNumber("right ultrasonic:", sonarDistance(rightSonar) );
    SmartDashboard.putNumber("left ultrasonic:", sonarDistance(leftSonar) );
  }
  public double poofs = 2.54;
  public void printLidarValues()
  {
    SmartDashboard.putNumber("test Lidar distance in", lidarDistance(testLidar)/poofs);
  }
  public void periodic()
  {
    getAngle();
    printPositions();
    printPower();
    printUltrasonicValues();
    printLidarValues();
  }
  public void stopMotors() {
    chassis.tankDrive(0, 0);
  }
  
  
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
