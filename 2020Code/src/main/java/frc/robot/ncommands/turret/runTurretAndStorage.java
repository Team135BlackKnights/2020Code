/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.ncommands.storage.runConveyorPower;
import frc.robot.nsubsystems.Storage;
import frc.robot.nsubsystems.Turret;
import frc.robot.nsubsystems.TurretLimelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class runTurretAndStorage extends ParallelCommandGroup {
  /**
   * Creates a new runTurretAndStorage.
   */
  

  public runTurretAndStorage(Storage _storage, Turret _turret, TurretLimelight _limelight) {
    super(
      new shootTurretDistance(_turret, _limelight),
      new runConveyorPower(_storage, -1800));
    



    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
  }
}
