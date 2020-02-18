/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.ncommands.drive.DriveWithTrajectory;
import frc.robot.ncommands.drive.shiftGears;
import frc.robot.ncommands.intake.moveIntake;
import frc.robot.ncommands.intake.runRoller;
import frc.robot.ncommands.turret.RotateTurretToAngle;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.nsubsystems.Intake;
import frc.robot.nsubsystems.Storage;
import frc.robot.nsubsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class leftAutonParallel extends ParallelCommandGroup {
  FalconDrive drive;
  Intake intake;
  Turret turret;

  public leftAutonParallel(FalconDrive drivesubsystem, Intake intakesubsystem, Turret turretsubsystem,
      Storage storagesubsystem) {
    super();

    drive = drivesubsystem;
    turret = turretsubsystem;
    intake = intakesubsystem;
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    parallel(new shiftGears(drive));
    if (drive.doesPathExist("PathWeaver/output/redRight.wpilib.json")) {
      parallel(new DriveWithTrajectory(drive, "PathWeaver/output/redRight.wpilib.json"));
    } else
     // parallel(new encoderDrive(drive, 90, 90));
    parallel(sequence(new runRoller(intake, .8)),
        sequence(parallel(new RotateTurretToAngle(turret, 90)), parallel(new moveIntake(intake))));

  }

}
