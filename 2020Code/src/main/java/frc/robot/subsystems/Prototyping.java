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
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.commands.prototyping.PrototypeManipControl;

public class Prototyping extends Subsystem implements RobotMap{
  public static Prototyping instance;

  public CANSparkMax
    topShooterSpark,
    bottomShooterSpark;

  public CANEncoder topEncoder, bottomEncoder;

  public DigitalInput photoSwitchTest;

  public TalonSRX 
    manipControlOneTalon, 
    manipControlTwoTalon, 
    buttonControlOneTalon, 
    buttonControlTwoTalon;

  public PigeonIMU pidgey;


  public Prototyping() {
    topShooterSpark = new CANSparkMax(MOTORS.REAR_LEFT_SPARK_ID, MotorType.kBrushless);
    bottomShooterSpark = new CANSparkMax(MOTORS.FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);

    ConfigSpark(topShooterSpark);
    ConfigSpark(bottomShooterSpark);

    manipControlOneTalon = new TalonSRX(MOTORS.MANIP_CONTROL_ONE_TALON);
    manipControlTwoTalon = new TalonSRX(MOTORS.MANIP_CONTROL_TWO_TALON);

    buttonControlOneTalon = new TalonSRX(MOTORS.BUTTON_CONTROL_ONE_TALON);
    buttonControlTwoTalon = new TalonSRX(MOTORS.BUTTON_CONTROL_TWO_TALON);
  
    ConfigTalon(manipControlOneTalon);
    ConfigTalon(manipControlTwoTalon);
    ConfigTalon(buttonControlOneTalon);
    ConfigTalon(buttonControlTwoTalon);

    topEncoder = new CANEncoder(topShooterSpark);
    bottomEncoder = new CANEncoder(bottomShooterSpark);

    pidgey = new PigeonIMU(0);
    resetPidgey();

    resetShooterEncoders();
    
    photoSwitchTest = new DigitalInput(SENSORS.SHOOTER_TRIP_ID);
    System.out.println("Prototyping Initialized");
  }

  public void resetPidgey()
  {
    pidgey.setYaw(0);
    pidgey.setFusedHeading(0);
  }

  double [] pidgeyYPR = new double[3];
  public double getYaw()
  {
    return pidgeyYPR[0];
  }
  
  public double getPitch()
  {
    return pidgeyYPR[1];
  }
  public double getRoll()
  {
    return pidgeyYPR[2];
  }
  public double getHeading()
  {
    return pidgey.getFusedHeading();
  }

  public boolean getDigitalInput(DigitalInput input)
  {
    return input.get();
  }
  public void ConfigSpark(CANSparkMax spark)
	{
		spark.setIdleMode(IdleMode.kBrake);
		spark.setInverted(false);
		spark.enableVoltageCompensation(12);
		
  }
  public void resetShooterEncoders()
  {
    topEncoder.setPosition(0);
    bottomEncoder.setPosition(0);
  }
  public double getEncoderTicks(CANEncoder encoder)
  {
    return encoder.getPosition();
  }

  public double getEncoderVelocity(CANEncoder encoder)
  {
    return encoder.getVelocity();
  }

  public double ticksToRevs(double ticks)
  {
    return ticks/4096;
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
    
    topShooterSpark.set(-topShooterPower);
    bottomShooterSpark.set(bottomShooterPower);

  }

  public double getMotorTemp(CANSparkMax spark)
  {
    return spark.getMotorTemperature();
  }

  public double neoPercentOutput(CANSparkMax spark)
  {
    return spark.getAppliedOutput();
  }
  public double neoVoltage(CANSparkMax spark)
  {
    return spark.getBusVoltage();
  }
  
  public void printSparkStuff()
  {
    SmartDashboard.putNumber("top Shooter Percent Output", neoPercentOutput(topShooterSpark));
    SmartDashboard.putNumber("Top Shooter Voltage", neoVoltage(topShooterSpark));
    
    SmartDashboard.putNumber("bottom Shooter Percent Output", neoPercentOutput(bottomShooterSpark));
    SmartDashboard.putNumber("bottom Shooter Voltage", neoVoltage(bottomShooterSpark));
  
    SmartDashboard.putNumber("Top Shooter Revs ", ticksToRevs(getEncoderTicks(topEncoder)));
    SmartDashboard.putNumber("Bottom Shooter Revs ", ticksToRevs(getEncoderTicks(bottomEncoder)));

    SmartDashboard.putNumber("top SHooter temp", getMotorTemp(topShooterSpark));
    SmartDashboard.putNumber("bottom shooter temp", getMotorTemp(bottomShooterSpark));
  }
  public void printDigitalInputs()
  {
    SmartDashboard.putBoolean("is photo swtich test tripped", getDigitalInput(photoSwitchTest));
  }
  public void printPidgey()
  {

    SmartDashboard.putNumber("pidgey yaw", getYaw());
    SmartDashboard.putNumber("pidgey ptich", getPitch());
    SmartDashboard.putNumber("pidgey roll", getRoll());
    SmartDashboard.putNumber("pidgey heading", getHeading());

  }
  @Override
  public void periodic() 
  {   // pidgey.getYawPitchRoll(pidgeyYPR);

   // printSparkStuff();
    //printDigitalInputs();
    //printPidgey();
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
