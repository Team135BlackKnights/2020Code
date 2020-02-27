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
import frc.robot.ncommands.intake.runRoller;
import frc.robot.ncommands.turret.runTurretAuton;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.nsubsystems.Intake;
import frc.robot.nsubsystems.Storage;
import frc.robot.nsubsystems.Turret;
import frc.robot.nsubsystems.TurretLimelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class middleAuto extends SequentialCommandGroup {
  /**
   * Creates a new middleAuto.
   */

  public middleAuto(FalconDrive drive, Turret turret, TurretLimelight limelight, Intake intake, Storage storage) 
  {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super
    (
      parallel(
        new runTurretAuton(turret, limelight, storage, 5),
      
      sequence(
        new leaveStartingConfig(intake, turret),
        parallel(
          new runRoller(intake, 3000, true),
          new encoderDrive(drive,2,3, true)
        ),
        parallel(
          new runRoller(intake, 3000, false),
          new encoderDrive(drive, 1, 1, false)
        )
      )

    ));
  }
}
