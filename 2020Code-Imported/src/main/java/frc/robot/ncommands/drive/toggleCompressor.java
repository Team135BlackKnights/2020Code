/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.FalconDrive;

public class toggleCompressor extends CommandBase {
  private static boolean isCompressorOn = true;
  FalconDrive drive;

  public toggleCompressor(FalconDrive subsystem) {
    drive = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Change the compressor to what it currently isn't
    isCompressorOn = !isCompressorOn;
    if (isCompressorOn) {
      drive.setCompressorOn();
    } else {
      drive.setCompressorOff();
    }
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
