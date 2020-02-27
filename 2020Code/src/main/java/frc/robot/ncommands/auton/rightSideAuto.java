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
import frc.robot.ncommands.turret.runTurretAuton;
import frc.robot.nsubsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class rightSideAuto extends SequentialCommandGroup {
  /**
   * Creates a new testAuto.
   */
  
  public rightSideAuto(FalconDrive drive, Intake intake, Turret turret, TurretLimelight limelight, Storage storage, boolean isShooting) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super
    (
      new leaveStartingConfig(intake, turret),
      parallel(
        new runTurretAuton(turret, limelight, storage, 6),
      sequence
      (
        new runRollerAndDriveRightSide(drive, intake),
        parallel(new encoderDrive(drive, 4, 4, false), new runRoller(intake, .65, false))
        )
      )
    );

    
    
  }
}
