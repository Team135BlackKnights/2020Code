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
import frc.robot.ncommands.turret.primeTurret;
import frc.robot.ncommands.turret.shootXBalls;
import frc.robot.nsubsystems.*;


public class rightSideAuto extends SequentialCommandGroup {
  
  public rightSideAuto(FalconDrive drive, Intake intake, Turret turret, Storage storage, boolean isShooting) {
    super
    (
      race(new leaveStartingConfig(intake, turret), new primeTurret(turret)),
      //new leaveStartingConfig(intake, turret),
  //    new encoderDrive(drive, .5, .5, false),
      new shootXBalls(turret , 3),
      race(new runRollerAndDriveRightSide(drive, intake), new primeTurret(turret)),
      //  new runRollerAndDriveRightSide(drive, intake),
      race(new encoderDrive(drive, 2, 2, false), new runRoller(intake, intake.autonRPM, false), new primeTurret(turret)),
      new shootXBalls(turret, 3)
      )
    ;
  }
}
