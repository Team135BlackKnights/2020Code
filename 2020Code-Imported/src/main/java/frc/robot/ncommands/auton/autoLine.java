/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ncommands.auton.parallels.leaveStartingConfig;
import frc.robot.ncommands.drive.encoderDrive;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.nsubsystems.Intake;
import frc.robot.nsubsystems.Turret;

public class autoLine extends SequentialCommandGroup {
  public autoLine(FalconDrive drive, Intake intake, Turret turret) {
   
    super
    (
      parallel(
        new leaveStartingConfig(intake, turret),
        new encoderDrive(drive, .5, .5, false))
    );
  }
}
