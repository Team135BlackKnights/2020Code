/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Turret;

public class RotateTurretToAngle extends CommandBase {
  public double _desiredAngle;
  Turret turret;
  
  
  double turretWheelDiameter = 12.8;
  double turretWheelCircumference = turretWheelDiameter * Math.PI;
  double turretWheelCurrentInches = 0;
  double turretWheelPercent = turretWheelCurrentInches / turretWheelCircumference;
  double encoderWheelDiameter = 1.4;
  double encoderWheelCircumference = encoderWheelDiameter * Math.PI;
  double encoderWheelCurrentInches = 0;
  double encoderWheelPercent = encoderWheelCurrentInches / encoderWheelCircumference;

  double currentAngle = 360 * turretWheelPercent;

  double rotationPower;
  double angleError;


  public RotateTurretToAngle(Turret subsystem, double Angle) {
    turret = subsystem;
    _desiredAngle = Angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encoderWheelCurrentInches = turret.ticksToInches(turret.rotationEncoder, encoderWheelDiameter);
    
    turretWheelCurrentInches = encoderWheelCurrentInches;
    turretWheelPercent = turretWheelCurrentInches / turretWheelCircumference;

    currentAngle = 360 * turretWheelPercent;


    angleError = Math.abs(currentAngle - _desiredAngle);
		boolean turnLeft  = currentAngle < _desiredAngle; 
    double turnModifer = turnLeft ? -1: 1;
    double minPower = .3;
    double P, I, D;
		P = .52;
		I = 0; 
		D = 0;

    rotationPower = (turnModifer * minPower)+ (angleError/90* P);
    turret.limit(rotationPower, .7, -.7);
    

    turret.rotationSpark.set(rotationPower);
    
    
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.rotationSpark.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  (Math.abs(angleError) <= 2);
  }
}
