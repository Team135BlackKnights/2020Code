/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton.parallels;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.ncommands.drive.encoderDrive;
import frc.robot.ncommands.intake.runRoller;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.nsubsystems.Intake;

public class runRollerAndDriveRightSide extends ParallelRaceGroup {
  public runRollerAndDriveRightSide(FalconDrive drive, Intake intake) {
    super
    (
      new runRoller(intake, intake.autonRPM, true),
      new encoderDrive(drive, 4.45 , 4.45, true)
    );
  }
}
