/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Turret;

public class RotateTurretToAngle extends CommandBase {
  private double _desiredAngle;
  Turret turret;

  // Using wheel size...
  double turretWheelDiameter = 12.8;
  // find the circumference of the wheel...
  double turretWheelCircumference = turretWheelDiameter * Math.PI;
  // find the current location of the turret
  double turretWheelCurrentInches = 0;
  // find the percentage based off the circumference and current location....
  double turretWheelPercent = turretWheelCurrentInches / turretWheelCircumference;

  // Do it again for the other wheel
  double encoderWheelDiameter = 1.4;
  double encoderWheelCircumference = encoderWheelDiameter * Math.PI;
  double encoderWheelCurrentInches = 0;
  double encoderWheelPercent = encoderWheelCurrentInches / encoderWheelCircumference;

  // Can use the percentage to find the current angle by mulitplying by 360
  double currentAngle = 360 * turretWheelPercent;

  double rotationPower;
  double angleError;
  boolean isFinished;

  // require input of subsystem and the desired angle
  public RotateTurretToAngle(Turret subsystem, double Angle) {

    // assign values to public locations
    turret = subsystem;
    _desiredAngle = Angle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Turret Command Running:", "Rotate turret to angle " + _desiredAngle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Going from encoder ticks to the angle of another wheel
    // encoderWheelCurrentInches = turret.ticksToInches(turret.rotationEncoder,
    // encoderWheelDiameter);
    encoderWheelCurrentInches = turret.getRotationTicks() * encoderWheelCircumference;
    turretWheelCurrentInches = encoderWheelCurrentInches;
    turretWheelPercent = turretWheelCurrentInches / turretWheelCircumference;
    currentAngle = 360 * turretWheelPercent;

    SmartDashboard.putNumber("turret wheel angle", currentAngle);

    // Angle error for how far off it currently is
    // TODO:: Test angle error now that the absolute value isn't being taken from it
    // so we can get a negative rotation
    angleError = _desiredAngle - currentAngle;
    // determine the direction of min power
    double turnModifer = angleError > 0 ? 1 : -1;

    // Variables for tuning
    double minPower = .3;
    double P;
    P = .52;

    // Find the overal power based off minimum power and the distance off then limit
    // it
    rotationPower = (turnModifer * minPower) + (angleError / 90 * P);
    // rotationPower = turret.limit(rotationPower, .7, -.7);
    isFinished = Math.abs(angleError) < 2;
    // set the power
    SmartDashboard.putNumber("Rotate Turret Rotation Power", rotationPower);
    turret.runRotation(rotationPower);
    SmartDashboard.putNumber("Turret rotation angle error", angleError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.rotationSpark.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
