/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton.parallels;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.ncommands.drive.resetDriveEncoders;
import frc.robot.ncommands.intake.moveIntake;
import frc.robot.ncommands.turret.setTurretPosPID;
import frc.robot.nsubsystems.Intake;
import frc.robot.nsubsystems.Turret;

public class leaveStartingConfig extends ParallelCommandGroup {
  public leaveStartingConfig(Intake intake, Turret turret) {
    super
    (
      //new moveIntake(intake),
      // 6 ball right side auto -150 position
      new moveIntake(intake),
      new setTurretPosPID(turret, 145, -150),
      new resetDriveEncoders(RobotContainer.drive)
    );
  }
}
