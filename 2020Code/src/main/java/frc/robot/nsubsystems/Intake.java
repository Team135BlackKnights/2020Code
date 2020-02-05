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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase implements RobotMap.INTAKE{
  /**
   * Creates a new Intake.
   */
  public CANSparkMax rollerSpark;
  public CANEncoder rollerEncoder;
  public Ultrasonic intakeSonar;
  public Solenoid raiseLower;

  public Intake() 
  {
    rollerSpark = new CANSparkMax(ROLLER_SPARK , MotorType.kBrushless);

    initCANSparkMax(rollerSpark, IdleMode.kCoast);

    rollerEncoder = rollerSpark.getEncoder();

    raiseLower = new Solenoid(RAISE_LOWER);
  
   // intakeSonar = new Ultrasonic(8, 9);

    //intakeSonar.setAutomaticMode(true);
    
    System.out.println("Intake Initialized");
  }

  public void initCANSparkMax(CANSparkMax spark, IdleMode mode)
  {
		spark.setInverted(false);
    spark.enableVoltageCompensation(12);
    spark.setIdleMode(mode);
  }

  public void runRoller(double power)
  {
    rollerSpark.set(power);
  }

  public double getEncoderPosition(CANEncoder encoder)
  {
    return encoder.getPosition();
  }

  public double ticksToRotations(double ticks)
  {
    return ticks/4096;
  }

  public double getEncoderVelocity(CANEncoder encoder)
  {
    return encoder.getVelocity();
  }

  public double getIntakeSonarDistanceIn()
  {
    return intakeSonar.getRangeInches();
  }
  
  public void raiseLower(boolean position)
  {
    raiseLower.set(position);
  }

  public boolean isRollerLowered()
  {
    return raiseLower.get();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
