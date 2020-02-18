/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ncommands.drive.encoderDrive;
import frc.robot.ncommands.drive.turnToAngle;
import frc.robot.ncommands.intake.moveIntake;
import frc.robot.ncommands.intake.runRoller;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.nsubsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class testAuto extends SequentialCommandGroup {
  /**
   * Creates a new testAuto.
   */
  public testAuto(FalconDrive drive, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new moveIntake(intake),
      parallel(new encoderDrive(drive, 4.45,4.45), new runRoller(intake, .4)),
      new encoderDrive(drive, 1.36, 1.36),
      new turnToAngle(drive, 50),
      parallel(new encoderDrive(drive, 4, 3.52), new runRoller(intake, .4)),
      new turnToAngle(drive, 0),
      new encoderDrive(drive, 2.68, 2.68)
    );

    
  }
}
