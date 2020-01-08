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

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.*;
import frc.robot.commands.PrototypeManipControl;

public class Prototyping extends Subsystem implements RobotMap.MOTORS{
  public static Prototyping instance;

  public TalonSRX 
    manipControlOneTalon, 
    manipControlTwoTalon, 
    buttonControlOneTalon, 
    buttonControlTwoTalon;


  public Prototyping() {
    manipControlOneTalon = new TalonSRX(MANIPCONTROLONE_TALON);
    manipControlTwoTalon = new TalonSRX(MANIPCONTROLTWO_TALON);

    buttonControlOneTalon = new TalonSRX(BUTTONCONTROLONE_TALON);
    buttonControlTwoTalon = new TalonSRX(BUTTONCONTROLTWO_TALON);
    

    manipControlOneTalon.setNeutralMode(NeutralMode.Brake);
    manipControlTwoTalon.setNeutralMode(NeutralMode.Brake);
    buttonControlOneTalon.setNeutralMode(NeutralMode.Brake);
    buttonControlTwoTalon.setNeutralMode(NeutralMode.Brake);

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
