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
import frc.robot.ncommands.turret.rotateAndTiltTurretToPos;
import frc.robot.nsubsystems.Intake;
import frc.robot.nsubsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class leaveStartingConfig extends ParallelCommandGroup {
  /**
   * Creates a new leaveStartingConfig.
   */
  public leaveStartingConfig(Intake intake, Turret turret) {
    super
    (
      new moveIntake(intake),
      new rotateAndTiltTurretToPos(turret, 80, 0),
      new resetDriveEncoders(RobotContainer.drive)
    );
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
  }
}
