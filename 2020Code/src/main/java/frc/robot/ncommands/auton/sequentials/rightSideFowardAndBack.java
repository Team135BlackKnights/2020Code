/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton.sequentials;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ncommands.drive.encoderDrive;
import frc.robot.nsubsystems.FalconDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class rightSideFowardAndBack extends SequentialCommandGroup {
  /**
   * Creates a new rightSideFowardAndBack.
   */
  public rightSideFowardAndBack(FalconDrive drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super
    (
      new encoderDrive(drive, 4.45, 4.45),
      new encoderDrive(drive, 1,1)
    );
  }
}
