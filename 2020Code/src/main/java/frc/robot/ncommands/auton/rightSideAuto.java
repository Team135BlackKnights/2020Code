/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ncommands.auton.parallels.leaveStartingConfig;
import frc.robot.ncommands.auton.parallels.runRollerAndDriveRightSide;
import frc.robot.ncommands.drive.encoderDrive;
import frc.robot.ncommands.intake.runRoller;
import frc.robot.nsubsystems.*;


public class rightSideAuto extends SequentialCommandGroup {
  
  public rightSideAuto(FalconDrive drive, Intake intake, Turret turret, Storage storage, boolean isShooting) {
    super
    (
      new leaveStartingConfig(intake, turret),
      parallel(
      sequence
      (
        new runRollerAndDriveRightSide(drive, intake),
        parallel(new encoderDrive(drive, 2, 2, false), new runRoller(intake, intake.autonRPM, false))
        )
      )
    );
  }
}
