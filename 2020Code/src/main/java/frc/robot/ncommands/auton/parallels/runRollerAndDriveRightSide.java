/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton.parallels;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.ncommands.auton.sequentials.rightSideFowardAndBack;
import frc.robot.ncommands.drive.encoderDrive;
import frc.robot.ncommands.intake.runRoller;
import frc.robot.nsubsystems.FalconDrive;
import frc.robot.nsubsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class runRollerAndDriveRightSide extends ParallelRaceGroup {
  /**
   * Creates a new runRollerAndDriveRightSide.
   */
  public runRollerAndDriveRightSide(FalconDrive drive, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super
    (
      new runRoller(intake, .4),
      new encoderDrive(drive, 4.45 , 4.45)
    );
  }
}
