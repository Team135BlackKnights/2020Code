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

  //Declares Motor Controllers and Chassis
  public SpeedControllerGroup leftDriveSide, rightDriveSide; 
  public DifferentialDrive chassis; 

  //Declares Ultrasonic Sensors
  public Ultrasonic rightSonar,leftSonar;
  public  Counter testLidar; 
  public AHRS navx;

  // If instance is empty, creates a new FalconDrive to fill it
    public static FalconDrive getInstance()
  {
     if (instance == null) {
        instance = new FalconDrive();
     } return instance;
     
  }
// Main FalconDrive system, runs when a new one is declared
  public FalconDrive()
  {
    //Creates each individual motor, named for its position on the robot
    frontLeftFX = new WPI_TalonFX(MOTORS.FRONT_LEFT_FALCON);
    rearLeftFX = new WPI_TalonFX(MOTORS.REAR_LEFT_FALCON);

    frontRightFX = new WPI_TalonFX(MOTORS.FRONT_RIGHT_FALCON);
    rearRightFX = new WPI_TalonFX(MOTORS.REAR_RIGHT_FALCON);
    
    // *************************************
    configFalcon(frontLeftFX, true);
    configFalcon(rearLeftFX, true);
    configFalcon(frontRightFX, true);
    configFalcon(rearRightFX, true);

    // Creates both Ultrasonic sensors
    leftSonar = new Ultrasonic(SENSORS.LEFT_SONAR_TRIG, SENSORS.LEFT_SONAR_ECHO);
    rightSonar = new Ultrasonic(SENSORS.RIGHT_SONAR_TRIG, SENSORS.RIGHT_SONAR_ECHO);

    //Sets the Ultrasonic Sensors so that they can function together
    rightSonar.setAutomaticMode(true);
    leftSonar.setAutomaticMode(true);

    // Initializes Lidar and runs its function
    testLidar = new Counter(SENSORS.FRONT_LIDAR_ID);
    initLidar(testLidar); 

    //Declares a new Navx and immediately sets it to 0
    navx = new AHRS(SENSORS.navXPort);
    navx.reset();

    // Organizes the individual motors into groups based on location
    leftDriveSide = new SpeedControllerGroup(frontLeftFX, rearLeftFX);
    rightDriveSide = new SpeedControllerGroup(frontRightFX, rearRightFX);
  
    // Declares the chassis as a DifferentialDrive, with the arguments of the motor controller groups
    chassis = new DifferentialDrive(leftDriveSide, rightDriveSide);

    
    chassis.setSafetyEnabled(false); //turns off system where if the motors don't recieve signal, the chassis learns about it and gets mad
    chassis.setMaxOutput(.98); // Maximum zoom is 98%
    
    resetEncoders(); //Calls method to reset the internal encoders of the motors
    setBrakeMode(NeutralMode.Brake); // Calls method which makes it so that when the input is neutral, the motors will brake
    
    System.out.println("Falcon Initialized"); // Outputs the text letting the user know that the Falcon has been initialized

  }

  // Configures the settings of the Talon Motors
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

  // Sets the neutral input to brake all four motors
  public void setBrakeMode(NeutralMode neutralMode)
  {
    frontLeftFX.setNeutralMode(neutralMode);
    frontRightFX.setNeutralMode(neutralMode);
    rearLeftFX.setNeutralMode(neutralMode);
    rearRightFX.setNeutralMode(neutralMode);

  }

  // Method sets the Chassis to Tank Drive mode while pulling double arguements
  public void TankDrive(double leftPower, double rightPower)
  {
    chassis.tankDrive(leftPower, rightPower);
  }

  // Method sets the Chassis to Arcade Drive while pulling double arguements
  public void ArcadeDrive(double lateralPower, double rotationalPower)
  { 
    chassis.arcadeDrive(lateralPower, -rotationalPower);
  }

  // Method sets the Chassis to Curvature Drive while pulling double arguements
  public void CurvatureDrive(double xSpeed, double zRotation)
  {
     chassis.curvatureDrive(xSpeed, zRotation, true);
  }
 
  // Method Resets all motor encoders to zero, then prints true on the smart dashboard
  public void resetEncoders()
  {
    frontLeftFX.setSelectedSensorPosition(0);
    frontRightFX.setSelectedSensorPosition(0);
    rearLeftFX.setSelectedSensorPosition(0);
    rearRightFX.setSelectedSensorPosition(0);
    SmartDashboard.putBoolean("Encoders Reset:", true);
  }

  //Takes the position of the encoder of a  specific motor and returns the value as the number of rotations (i.e. 1.5 or 2)
  public double getEncoderDistance(TalonFX falcon)
  {
    return falcon.getSelectedSensorPosition()/4096;
  }

  //Returns the velocity of the chosen motor's encoder
  public double getEncoderVelocity(TalonFX falcon)
  {
    return falcon.getSelectedSensorVelocity();
  }

  //Function to initialize the Lidar, Set the max period where it is still considered moving to 1 second
  public void initLidar(Counter lidar)
  {
    lidar.setMaxPeriod(1.00);
    lidar.setSemiPeriodMode(true);
    lidar.reset();
  }

  // Determines the distance being read by the lidar
  public double lidarDistance(Counter lidar)
  {
    double lidarDistance;
    if(lidar.get() < 1)
      lidarDistance = 0;
    else 
      lidarDistance = (lidar.getPeriod() * 100000.0);
    return lidarDistance;
  }

  //Prints on the dashboard the current angle of the robot based on the navx's Yaw reading
  public double getAngle()
  {
   SmartDashboard.putNumber("Current angle of Robot:", navx.getYaw());
   return navx.getYaw();
 }

 //Returns the rate at which the robot is rotating(in degrees per second) based on the navx's yaw reading
 public double getRotationRate()
 {
   return navx.getRate();
 }

 // Resets the Yaw of the navx
  public void resetGyro()
  {
    navx.zeroYaw();
  }

  //Tests if the variable x is above the upper limit or below the lower, and if so, it sets it to the limit
    public double limit(double x, double upperLimit, double lowerLimit)
	{	if(x >= upperLimit){ x = upperLimit;}
		else if( x<=lowerLimit){ x = lowerLimit;}
		return x;
	}

  //Finds the Left position of the encoders of the left drive group, and averages them in order to normalize the results
     public double getLeftPos()
  {
    return (getEncoderDistance(frontLeftFX) + getEncoderDistance(rearLeftFX))/2;
  }

    //Finds the right position of the encoders of the right drive group, and averages them in order to normalize the results
  public double getRightPos()
  {
    return (getEncoderDistance(frontRightFX) + getEncoderDistance(rearRightFX))/2;
  }

  //Prints the positions of all of encoders as well as the averages of the groups
    public void printPositions()
  {
    SmartDashboard.putNumber("front Left Position:", getEncoderDistance(frontLeftFX));
    SmartDashboard.putNumber("rear Left Position:", getEncoderDistance(rearLeftFX));
    SmartDashboard.putNumber("front Right Position:", getEncoderDistance(frontRightFX));
    SmartDashboard.putNumber("rear Right Position:", getEncoderDistance(rearRightFX));
    
    SmartDashboard.putNumber("Average Left Position:", getLeftPos());
    SmartDashboard.putNumber("Average Right Position:", getRightPos());

    
  } 
  
  //Returns the amount of power being output by the motor controller
  public double getMotorPercent(WPI_TalonFX falcon)
  {
    return falcon.getMotorOutputPercent();
  }

  //Returns the amount of voltage being output by the motor controller
  public double getMotorVoltage(WPI_TalonFX falcon)
  {
    return falcon.getMotorOutputVoltage();
  }

  //Returns the temperature of the motor controller
  public double getMotorTemp(WPI_TalonFX falcon)
 {
  return falcon.getTemperature();
 }

 //Prints the results of the getmotorpercent method for all of the four motors
 public void printPower()
  {
    SmartDashboard.putNumber("front Left Output Percent: ", getMotorPercent(frontLeftFX));
    SmartDashboard.putNumber("rear Left Output Percent: ", getMotorPercent(rearLeftFX));
    SmartDashboard.putNumber("front Right Output Percent: ", getMotorPercent(frontRightFX));
    SmartDashboard.putNumber("rear Right Output Percent: ", getMotorPercent(rearRightFX));
  }

   //Prints the results of the getmotorvoltage method for all of the four motors
  public void printVoltage()
  {
    SmartDashboard.putNumber("front Left Voltage: ", getMotorVoltage(frontLeftFX));
    SmartDashboard.putNumber("rear Left Voltage: ", getMotorVoltage(rearLeftFX));
    SmartDashboard.putNumber("front Right Voltage: ", getMotorVoltage(frontRightFX));
    SmartDashboard.putNumber("rear Right Voltage: ", getMotorVoltage(rearRightFX));
  }

  //Prints the results of the getmotortemp method for all of the four motors
  public void printTemperature()
  {
    SmartDashboard.putNumber("front Left Temp: ", getMotorTemp(frontLeftFX));
    SmartDashboard.putNumber("rear Left Temp: ", getMotorTemp(rearLeftFX));
    SmartDashboard.putNumber("front Right Temp: ", getMotorTemp(frontRightFX));
    SmartDashboard.putNumber("rear Right Temp: ", getMotorTemp(rearRightFX));
  }
  
  //Returns the distance found by the Ultrasonic Sensors in inches
  public double sonarDistance(Ultrasonic sonar)
  {
    return sonar.getRangeInches();
  }

  //Prints the distances found by each ultrasonic sensor in inches
  public void printUltrasonicValues() {
    SmartDashboard.putNumber("right ultrasonic:", sonarDistance(rightSonar) );
    SmartDashboard.putNumber("left ultrasonic:", sonarDistance(leftSonar) );
  }
  public double poofs = 2.54; //Constant for conversion between inches and centimeters

  //Prints the distance found by the lidar in inches
  public void printLidarValues()
  {
    SmartDashboard.putNumber("test Lidar distance in", lidarDistance(testLidar)/poofs);
  }

  //Periodic Loop
  public void periodic()
  {
    getAngle(); //Finds the angle of the robot based on the Navx
    printPositions(); //Prints the positions of all the encoders
    printPower(); //Prints the power output of the motor controllers
    printUltrasonicValues(); //Prints the distances found by the ultrasonic sensors
    printLidarValues(); //Prints the distances found by the Lidars
  }

  //Sets the power of the motors to zero, causing the robot to stop
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
