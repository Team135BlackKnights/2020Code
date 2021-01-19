/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ncommands.auton.parallels.leaveStartingConfig;
import frc.robot.ncommands.drive.EncoderDrive;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.nsubsystems.Intake;
import frc.robot.nsubsystems.Storage;
import frc.robot.nsubsystems.Turret;


public class autoLinePlus extends SequentialCommandGroup {
  
  public autoLinePlus(FalconDrive drive, Intake intake, Turret turret, Storage storage) {

    super(
      parallel(
        new leaveStartingConfig(intake, turret),
        new EncoderDrive(drive, -1.25, -1.25, false)
        )
    );
  }
}
