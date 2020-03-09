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

public class setTurretPosPID extends CommandBase {
  /**
   * Creates a new setTurretPosPID.
   */
  Turret turret;
  double desiredHoodPos, desiredRotationPos;
  boolean isFinished;

  public setTurretPosPID(Turret _turret, double _desiredHoodPos, double _desiredRotationPos) {
    turret = _turret;
    desiredHoodPos = _desiredHoodPos;
    desiredRotationPos = _desiredRotationPos;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    SmartDashboard.putString("New Turret Command Running: ", "set Turret To Pos w/PID");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Find how far off the hood and roatation are
    double currentHoodPos, currentRotationPos, hoodError, rotationError, hP, rP, hoodInput, rotationInput;
    currentHoodPos = turret.getHoodPos();
    currentRotationPos = turret.getRotationPos();
    hoodError = desiredHoodPos - currentHoodPos;
    rotationError = desiredRotationPos - currentRotationPos;

    // Set tolerance
    isFinished = (Math.abs(hoodError) < 2 && Math.abs(rotationError) < 1);

    // Set tuning variables
    hP = 1;
    rP = 1;

    // Set powers
    hoodInput = hoodError * hP;
    rotationInput = rotationError * rP;

    turret.aimTurret(rotationInput, hoodInput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
