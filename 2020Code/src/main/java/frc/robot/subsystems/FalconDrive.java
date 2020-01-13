/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class FalconDrive extends Subsystem implements RobotMap{

  public FalconDrive instance; 
  public TalonFX leftFrontFX, rightFrontFX, leftRearFX, rightRearFX;

  public SpeedControllerGroup leftDriveSide, rightDriveSide; 
  public DifferentialDrive chassis; 
  /*
  public AHRS navx;
  public Counter leftLidar, rightLidar;
  public Ultrasonic leftSonar, rightSonar, rearSonar, frontSonar;
  */
  public FalconDrive getInstance()
  {
     if (instance == null) {
        instance = new FalconDrive();
     } return instance;
  }
  private FalconDrive()
  {
    configFalcon(leftFrontFX, true, MOTORS.FRONT_LEFT_FALCON);
    configFalcon(leftRearFX, true, MOTORS.REAR_LEFT_FALCON);
    configFalcon(rightFrontFX, false, MOTORS.FRONT_RIGHT_FALCON);
    configFalcon(rightRearFX, false, MOTORS.REAR_RIGHT_FALCON);

    chassis = new DifferentialDrive(leftDriveSide, rightDriveSide);
    chassis.setSafetyEnabled(false);
    chassis.setMaxOutput(.98);
  }

  public void configFalcon(TalonFX falcon, boolean isLeft, int ID)
  { 
    falcon = new TalonFX(ID);
    falcon.setNeutralMode(NeutralMode.Brake);
    falcon.enableVoltageCompensation(true);
  }
  public void ResetEncoders()
  {
    leftFrontFX.setSelectedSensorPosition(0);
    rightFrontFX.setSelectedSensorPosition(0);
    leftRearFX.setSelectedSensorPosition(0);
    rightRearFX.setSelectedSensorPosition(0);
  }

  public double getEncoderDistance(TalonFX falcon)
  {
    return falcon.getSelectedSensorPosition();
  }
  public double getEncoderVelocity(TalonFX falcon)
  {
    return falcon.getSelectedSensorVelocity();
  }

  public double getLeftPos()
  {
    return (getEncoderDistance(leftFrontFX) + getEncoderDistance(leftRearFX))/2;
  }
  public double getRightPos()
  {
    return (getEncoderDistance(rightFrontFX) + getEncoderDistance(rightRearFX))/2;
  }

  public void forceBreakMode(boolean forceBreak)
  {
    SmartDashboard.putBoolean("Forced Break Mode:", forceBreak);
     if(forceBreak)
     { 
       leftFrontFX.setNeutralMode(NeutralMode.Brake);
       rightFrontFX.setNeutralMode(NeutralMode.Brake);
       leftRearFX.setNeutralMode(NeutralMode.Brake);
       rightRearFX.setNeutralMode(NeutralMode.Brake);
     }
  }

  public void TankDrive(double leftPower, double rightPower)
  {
    chassis.tankDrive(leftPower, rightPower);
  }

  public void ArcadeDrive(double lateralPower, double rotationalPower)
  { 
    chassis.arcadeDrive(lateralPower, rotationalPower);
  }
  public void CurvatureDrive(double xSpeed, double zRotation)
  {
     chassis.curvatureDrive(xSpeed, zRotation, true);
  }
  public void EncoderDrive(double leftPos, double rightPos)
  {
    leftFrontFX.set(ControlMode.Position, leftPos);
    leftRearFX.set(ControlMode.Position, leftPos);
    rightFrontFX.set(ControlMode.Position, rightPos);
    rightFrontFX.set(ControlMode.Position, rightPos);
  }

  public void printPositions()
  {
    SmartDashboard.putNumber("leftFront Position", getEncoderDistance(leftFrontFX));
    SmartDashboard.putNumber("leftRear Position", getEncoderDistance(leftRearFX));
  } 
  
  
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
