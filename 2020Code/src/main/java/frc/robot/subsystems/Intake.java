/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem implements RobotMap.INTAKE{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static Intake instance; 
  public CANSparkMax rollerSpark, conveyorSpark;
  public CANEncoder rollerEncoder, conveyorEncoder;
  public DigitalInput intakeBallTripSwitch;
  public Ultrasonic intakeSonar;
  public int currentBallCount;
  public boolean lastSwtichPosition;
  public Solenoid raiseLower;
  public Compressor compressor;

  public static Intake getInstance(){
  if(instance == null)
  {
    instance = new Intake();
  }
  return instance;
  }

  public Intake()
  {
    rollerSpark = new CANSparkMax(ROLLER_SPARK , MotorType.kBrushless);
    conveyorSpark = new CANSparkMax(CONVEYOR_SPARK, MotorType.kBrushless);

    initCANSparkMax(rollerSpark, IdleMode.kCoast);
    initCANSparkMax(conveyorSpark, IdleMode.kBrake);

    rollerEncoder = rollerSpark.getEncoder();
    conveyorEncoder = conveyorSpark.getEncoder();

    raiseLower = new Solenoid(RAISE_LOWER);
    compressor = new Compressor();

    compressor.setClosedLoopControl(true);
    compressor.start();


    intakeBallTripSwitch = new DigitalInput(INTAKE_TRIP_SWITCH);
    intakeSonar = new Ultrasonic(INTAKE_SONAR_TRIG, INTAKE_SONAR_ECHO);

    intakeSonar.setAutomaticMode(true);
    lastSwtichPosition = false;
    currentBallCount = 0;
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

  public void runConveyor(double power)
  {
    conveyorSpark.set(power);
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

  public double getConveyorTravelDistance()
  {
    return ticksToRotations(getEncoderPosition(conveyorEncoder)) * Math.PI * 4;
  }

  public void raiseLower(boolean position)
  {
    raiseLower.set(position);
  }

  public void setCompressorOff()
  {
    compressor.setClosedLoopControl(false);
    compressor.stop();
  }

  public void setCompressorOn()
  {
    compressor.setClosedLoopControl(true);
  }

  public boolean isRollerLowered()
  {
    return raiseLower.get();
  }

  public boolean isCompressorOn()
  {
    return compressor.getClosedLoopControl();
  }

  
  public boolean isBallAtTripSwitch()
  {
    return intakeBallTripSwitch.get();
  }

  public double getIntakeSonarDistanceIn()
  {
    return intakeSonar.getRangeInches();
  }

  public void intakeBallCount()
  {
      if(isBallAtTripSwitch()!= lastSwtichPosition && isBallAtTripSwitch()!=false)
      {
        currentBallCount++;
      }
      if(lastSwtichPosition!=isBallAtTripSwitch())
      {
        lastSwtichPosition = isBallAtTripSwitch();
      }
  }

  public void periodic()
  {
    intakeBallCount();
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
