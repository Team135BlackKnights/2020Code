/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.storage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Storage;
import frc.robot.util.MotorControl;
import frc.robot.util.PathFIndingFIles.MovingAverage;

public class runConveyorPower extends CommandBase {
  /**
   * Creates a new runConveyorPower.
   */
  Storage storage;
  private double RPM;
  private double previousError;
  public runConveyorPower(Storage _storage, double _RPM) {
    storage = _storage;
    RPM = _RPM;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Storage Command Running: ", " conveyorPID "+ RPM);
    previousError = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentRPM = MotorControl.getMotorVelocity(storage.conveyorEncoder);
    MovingAverage smoothPls = new MovingAverage(25);
    double maybe = smoothPls.process((float)currentRPM);
    double storageMaxVel = 5200;

    double error = RPM-maybe;
    error = error/storageMaxVel;

    double integral =+error*.02;
    double derivative = (error-previousError)/.02;
    
    double kP, kI, kD;
    kP = 2.1;
    kI = 0;
    kD = 0;

    double storageInput = error*kP + integral *kI + derivative * kD;
    storageInput  = 0;//RobotContainer.turret.isShooterUpToSpeed ? storageInput : 0; 
    storage.runConveyor(storageInput);
    previousError = error;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    SmartDashboard.putString("Storage Command Running: ", "No Command Running");
    storage.runConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
