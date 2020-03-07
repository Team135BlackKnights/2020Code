/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.nsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.util.MotorControl;

public class Intake extends SubsystemBase implements RobotMap.INTAKE {
  public CANSparkMax rollerSpark;
  public CANEncoder rollerEncoder;
  public Solenoid raiseLower;

  public boolean raiseLowerState;
  public double autonRPM;

  public Intake() {
    // Motor and Encoder setup
    rollerSpark = new CANSparkMax(ROLLER_SPARK, MotorType.kBrushless);
    MotorControl.initCANSparkMax(rollerSpark, false, false, 30);
    rollerEncoder = rollerSpark.getEncoder();

    // Solenoid declaration
    raiseLower = new Solenoid(RAISE_LOWER);

    // Default RPM for rollers during auto
    autonRPM = 4800;

    // Intake initialized
    System.out.println("Intake Initialized");
  }

  // Run our intake
  public void runRoller(double power) {
    rollerSpark.set(power);
  }

  // Set intake to lowered position
  public void raiseLower(boolean position) {
    raiseLower.set(position);
  }

  // Check position of intake
  public boolean isRollerLowered() {
    return raiseLower.get();
  }

  // Print useful data to help drivers
  public void printIntakeData() {
    SmartDashboard.putBoolean("is Intake Lower", isRollerLowered());
    SmartDashboard.putNumber("intake roller position", MotorControl.getSparkEncoderPosition(rollerEncoder));
    SmartDashboard.putNumber("roller Velocity", MotorControl.getSparkVelocity(rollerEncoder));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("roller Velocity", MotorControl.getSparkVelocity(rollerEncoder));

    // printIntakeData();
  }
}
