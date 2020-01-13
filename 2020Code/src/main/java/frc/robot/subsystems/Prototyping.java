/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.*;
import frc.robot.commands.PrototypeManipControl;

public class Prototyping extends Subsystem implements RobotMap.MOTORS{
  public static Prototyping instance;

  public CANSparkMax
    topShooterSpark,
    bottomShooterSpark;

  public TalonSRX 
    manipControlOneTalon, 
    manipControlTwoTalon, 
    buttonControlOneTalon, 
    buttonControlTwoTalon;


  public Prototyping() {
    topShooterSpark = new CANSparkMax(FRONT_LEFT_SPARK_ID, MotorType.kBrushless);
    bottomShooterSpark = new CANSparkMax(FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);

    ConfigSpark(topShooterSpark);
    ConfigSpark(bottomShooterSpark);

    manipControlOneTalon = new TalonSRX(MANIPCONTROLONE_TALON);
    manipControlTwoTalon = new TalonSRX(MANIPCONTROLTWO_TALON);

    buttonControlOneTalon = new TalonSRX(BUTTONCONTROLONE_TALON);
    buttonControlTwoTalon = new TalonSRX(BUTTONCONTROLTWO_TALON);
  
    ConfigTalon(manipControlOneTalon);
    ConfigTalon(manipControlTwoTalon);
    ConfigTalon(buttonControlOneTalon);
    ConfigTalon(buttonControlTwoTalon);

  }
  public void ConfigSpark(CANSparkMax spark)
	{
		spark.setIdleMode(IdleMode.kBrake);
		spark.setInverted(false);
		spark.enableVoltageCompensation(12);
		
	}

  public void ConfigTalon(TalonSRX talon)
  {
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configReverseSoftLimitEnable(false);
    talon.configForwardSoftLimitEnable(false);
    talon.enableCurrentLimit(true);
    talon.enableVoltageCompensation(true);
  }

  public void runManip(double power) {
    manipControlOneTalon.set(ControlMode.PercentOutput, power);
    manipControlTwoTalon.set(ControlMode.PercentOutput, power);
  }
  public void runButtonTalon1(double power)
  {
    buttonControlOneTalon.set(ControlMode.PercentOutput, power);
  }
  public void runButtonTalon2(double power)
  {
    buttonControlTwoTalon.set(ControlMode.PercentOutput, power);
  }

  public void runShooter(double topShooterRPM, double bottomShooterRPM) {
    double volts = 1;
    int maxRPM = 5676;
    
    double topShooterPower = (volts/maxRPM) * topShooterRPM;
    double bottomShooterPower = (volts/maxRPM) * bottomShooterRPM;
    
    topShooterSpark.set(topShooterPower);
    bottomShooterSpark.set(bottomShooterPower);

  }
  @Override
  public void periodic() {


    // This method will be called once per scheduler run
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new PrototypeManipControl());
  }

  public static Prototyping getInstance() {
    if (instance == null) {
      instance = new Prototyping();
    }
    return instance;
  }

}
