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
import frc.robot.ncommands.intake.runRoller;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.nsubsystems.Intake;
import frc.robot.nsubsystems.Storage;
import frc.robot.nsubsystems.Turret;

public class middleAuto extends SequentialCommandGroup {

  public middleAuto(FalconDrive drive, Turret turret, Intake intake, Storage storage) 
  {
    super
    (
      
      sequence(
        new leaveStartingConfig(intake, turret),
        parallel(
          new runRoller(intake, 3000, true),
          new EncoderDrive(drive,2,3, true)
        ),
        parallel(
          new runRoller(intake, 3000, false),
          new EncoderDrive(drive, 1, 1, false)
        )
      )

    );
  }
}
