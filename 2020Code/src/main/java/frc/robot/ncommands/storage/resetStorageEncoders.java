/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Storage;
import frc.robot.util.MotorControl;

//This command is made for reseting the encoder's position in the storage subsystem

public class resetStorageEncoders extends CommandBase {
  Storage storage;

  public resetStorageEncoders(Storage subsystem) {
    storage = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // storage.resetConveyorEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Reset the encoder for the Conveyor in storage
    MotorControl.resetSparkEncoder(storage.conveyorEncoder);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
