/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

public class FalconDrive extends SubsystemBase implements RobotMap.DRIVE {
  public double poofs = 2.54; // Constant for conversion between inches and centimeters
  public static final double distBetweenWheelsInches = 21;// 26.84603809585759;
  public static final double gearRatio = 1/10.86;
  public static final double wheelDiameterInches = 6.375;// 18;
  public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
  public static final double encoderTicksPerRev = 2048;
 // DifferentialDriveOdometry(getHeading());

  public DifferentialDriveOdometry m_odometry;

  public Pose2d pose;

  public WPI_TalonFX frontLeftFX, frontRightFX, rearLeftFX, rearRightFX;
  public Solenoid shifter;
  public Compressor compressor;

  public SpeedControllerGroup leftDriveSide, rightDriveSide;
  public DifferentialDrive chassis;

  // Declares Ultrasonic Sensors
  public AHRS navx;

  public FalconDrive() {
    // Creates each individual motor, named for its position on the robot
    frontLeftFX = new WPI_TalonFX(FRONT_LEFT_FALCON);
    rearLeftFX = new WPI_TalonFX(REAR_LEFT_FALCON);

    frontRightFX = new WPI_TalonFX(FRONT_RIGHT_FALCON);
    rearRightFX = new WPI_TalonFX(REAR_RIGHT_FALCON);

    // *************************************
    configFalcon(frontLeftFX, true);
    configFalcon(rearLeftFX, true);
    configFalcon(frontRightFX, true);
    configFalcon(rearRightFX, true);

    shifter = new Solenoid(SHIFTER_ID);
    compressor = new Compressor();

    compressor.setClosedLoopControl(false);
    compressor.start();

    // Declares a new Navx and immediately sets it to 0
    navx = new AHRS(navXPort);
    navx.reset();

    // Organizes the individual motors into groups based on location
    leftDriveSide = new SpeedControllerGroup(frontLeftFX, rearLeftFX);
    rightDriveSide = new SpeedControllerGroup(frontRightFX, rearRightFX);

    // Declares the chassis as a DifferentialDrive, with the arguments of the motor
    // controller groups
    chassis = new DifferentialDrive(leftDriveSide, rightDriveSide);
    chassis.setSafetyEnabled(false);
    // turns off system where if the motors don't recieve signal, the chassis learns
    // about it and gets mad

    chassis.setMaxOutput(.98); // Maximum zoom is 98%

    resetEncoders(); // Calls method to reset the internal encoders of the motors
    setBrakeMode(NeutralMode.Coast);
    // Calls method which makes it so that when the input is neutral, the motors
    // will brake
   
    pose = new Pose2d();
    m_odometry = new DifferentialDriveOdometry(getHeading());


    System.out.println("Falcon Drive Initialized");
  }

  public void configFalcon(WPI_TalonFX falcon, boolean isLeft) {
    falcon.setNeutralMode(NeutralMode.Brake);
    falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
    falcon.configVoltageCompSaturation(12.0, 100);
    falcon.enableVoltageCompensation(true);
    falcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 100);
    falcon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 100);
    falcon.configVelocityMeasurementWindow(4, 100);
    falcon.configClosedloopRamp(.5);
    falcon.configOpenloopRamp(.5);
  }

  // Sets the neutral input to brake all four motors
  public void setBrakeMode(NeutralMode neutralMode) {
    frontLeftFX.setNeutralMode(neutralMode);
    frontRightFX.setNeutralMode(neutralMode);
    rearLeftFX.setNeutralMode(neutralMode);
    rearRightFX.setNeutralMode(neutralMode);

  }

  // Method sets the Chassis to Tank Drive mode while pulling double arguements
  public void TankDrive(double leftPower, double rightPower) {
    chassis.tankDrive(leftPower, rightPower);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftMps(), getRightMps());
  }

  public void setVoltageCompensation(boolean isDesired)
  {
    frontLeftFX.enableVoltageCompensation(isDesired);
    rearLeftFX.enableVoltageCompensation(isDesired);
    frontRightFX.enableVoltageCompensation(isDesired);
    rearRightFX.enableVoltageCompensation(isDesired);
  }

  // Method sets the Chassis to Arcade Drive while pulling double arguements
  public void ArcadeDrive(double lateralPower, double rotationalPower) {
    chassis.arcadeDrive(lateralPower, rotationalPower);
  }

  public void setCompressorOff() {
    compressor.setClosedLoopControl(false);
    compressor.stop();
  }

  public void stopMotors() {
    chassis.tankDrive(0, 0);
  }

  public void setCompressorOn() {
    compressor.setClosedLoopControl(true);
  }

  public boolean isCompressorOn() {
    return compressor.getClosedLoopControl();
  }

  // Method Resets all motor encoders to zero, then prints true on the smart
  // dashboard
  public void resetEncoders() {
    frontLeftFX.setSelectedSensorPosition(0);
    frontRightFX.setSelectedSensorPosition(0);
    rearLeftFX.setSelectedSensorPosition(0);
    rearRightFX.setSelectedSensorPosition(0);
  }

  // Takes the position of the encoder of a specific motor and returns the value
  // as the number of rotations (i.e. 1.5 or 2)
  public double getEncoderDistance(TalonFX falcon) {
    return falcon.getSelectedSensorPosition() / 4096;
  }

  // Returns the velocity of the chosen motor's encoder
  public double getEncoderVelocity(TalonFX falcon) {
    return falcon.getSelectedSensorVelocity()*10/4096;
  }
  

  public void shiftGears(boolean isHighGear) {
    shifter.set(isHighGear);
  }
  
  public boolean shifterState()
  {
    return shifter.get();

  }


  // Resets the Yaw of the navx
  public void resetGyro() {
    navx.zeroYaw();
  }

  // Tests if the variable x is above the upper limit or below the lower, and if
  // so, it sets it to the limit
  public double limit(double x, double upperLimit, double lowerLimit) {
    if (x >= upperLimit) {
      x = upperLimit;
    } else if (x <= lowerLimit) {
      x = lowerLimit;
    }
    return x;
  }

  // Finds the Left position of the encoders of the left drive group, and averages
  // them in order to normalize the results
  public double getLeftPos() {
    return (getEncoderDistance(frontLeftFX) + getEncoderDistance(rearLeftFX)) / 2;
  }

  // Finds the right position of the encoders of the right drive group, and
  // averages them in order to normalize the results
  public double getRightPos() {
    return (-getEncoderDistance(frontRightFX) + -getEncoderDistance(rearRightFX)) / 2;
  }
  
  public double rotationsToInches(double rotations)
  {
    return rotations*6*Math.PI;
  }
  
  public double getLeftMetres()
  {
    return (Units.inchesToMeters(rotationsToInches(getLeftPos()))) * gearRatio;
  }
  public double getRightMetres()
  {
    return (Units.inchesToMeters(rotationsToInches(getRightPos()))) * gearRatio;
  }

  public void printMetres()
  {
    SmartDashboard.putNumber("left Dist Metres", getLeftMetres());
    SmartDashboard.putNumber("right Dist Metres", getRightMetres());
  }

  public double getEncoderMps(TalonFX talon)
  {
    return 0.0254*(rotationsToInches(getEncoderVelocity(talon))) * gearRatio;
  }
  
  public double getLeftMps()
  {
    return (getEncoderMps(frontLeftFX) + getEncoderMps(rearLeftFX))/2;
  }

  public double getRightMps()
  {
    return (-getEncoderMps(frontRightFX) + -getEncoderMps(rearRightFX))/2;
  }

  public double getLinearMps()
  {
    return (getLeftMps() + getRightMps())/2;
  }

  public double getAngularMps()
  {
    return (getLeftMps() - getRightMps())/Units.inchesToMeters(21);
  }
  
  // Prints the positions of all of encoders as well as the averages of the groups
  public void printPositions() {
    SmartDashboard.putNumber("Average Left Position:", getLeftPos());
    SmartDashboard.putNumber("Average Right Position:", getRightPos());
  }

  // Returns the amount of power being output by the motor controller
  public double getMotorPercent(WPI_TalonFX falcon) {
    return falcon.getMotorOutputPercent();
  }

  // Returns the amount of voltage being output by the motor controller
  public double getMotorVoltage(WPI_TalonFX falcon) {
    return falcon.getMotorOutputVoltage();
  }

  // Returns the temperature of the motor controller
  public double getMotorTemp(WPI_TalonFX falcon) {
    return falcon.getTemperature();
  }

  // Prints the results of the getmotorpercent method for all of the four motors
  public void printPower() {
    SmartDashboard.putNumber("front Left Output Percent: ", getMotorPercent(frontLeftFX));
    SmartDashboard.putNumber("rear Left Output Percent: ", getMotorPercent(rearLeftFX));
    SmartDashboard.putNumber("front Right Output Percent: ", getMotorPercent(frontRightFX));
    SmartDashboard.putNumber("rear Right Output Percent: ", getMotorPercent(rearRightFX));
  }

  // Prints the results of the getmotorvoltage method for all of the four motors
  public void printVoltage() {
    SmartDashboard.putNumber("front Left Voltage: ", getMotorVoltage(frontLeftFX));
    SmartDashboard.putNumber("rear Left Voltage: ", getMotorVoltage(rearLeftFX));
    SmartDashboard.putNumber("front Right Voltage: ", getMotorVoltage(frontRightFX));
    SmartDashboard.putNumber("rear Right Voltage: ", getMotorVoltage(rearRightFX));
  }
  

  // Prints the results of the getmotortemp method for all of the four motors
  public void printTemperature() {
    SmartDashboard.putNumber("front Left Temp: ", getMotorTemp(frontLeftFX));
    SmartDashboard.putNumber("rear Left Temp: ", getMotorTemp(rearLeftFX));
    SmartDashboard.putNumber("front Right Temp: ", getMotorTemp(frontRightFX));
    SmartDashboard.putNumber("rear Right Temp: ", getMotorTemp(rearRightFX));
  }

  // Returns the distance found by the Ultrasonic Sensors in inches

  public void printMps()
  {
    SmartDashboard.putNumber("Left Mps", getLeftMps());
    SmartDashboard.putNumber("Right Mps", getRightMps());
    SmartDashboard.putNumber("Linear Mps ", getLinearMps());
    SmartDashboard.putNumber("Angular Mps", getAngularMps());

  }

  public void printPose()
  {
    SmartDashboard.putNumber("rotation", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("translation y ", pose.getTranslation().getY());
    SmartDashboard.putNumber("translation x ", pose.getTranslation().getX());
  }


  // Prints the distance found by the lidar in inches

  @Override
  public void periodic() 
  {
   
    pose = m_odometry.update(getHeading(),getLeftMetres(), getRightMetres());
   // SmartDashboard.putString("pose", pose.toString());
    //printPose();
    //printMps();
    //printMetres();
    // This method will be called once per scheduler run
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public Rotation2d getHeading() 
  {
    return Rotation2d.fromDegrees(Math.IEEEremainder(-navx.getYaw(), 360.0d));
  }

  
  public double robotXPos()
  {
    return pose.getTranslation().getX();
  }

  public double robotYPos()
  {
    return pose.getTranslation().getY();
  }

  public double robotTheta()
  {
    return pose.getRotation().getDegrees();
  }

  public boolean drivingForward()
  {
    return getLinearMps() > 0 && (getAngularMps() <=.15);
  }

  public boolean drivingBackward()
  {
    return getLinearMps() < 0 && (getAngularMps() <=.15);
  }

  
    public void resetOdometry() { m_odometry.resetPosition(new Pose2d(),
    getHeading()); }
     
}
