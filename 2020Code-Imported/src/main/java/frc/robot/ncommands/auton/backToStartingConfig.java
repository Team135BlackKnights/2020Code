/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.auton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ncommands.turret.TurnOffLimelight;
import frc.robot.ncommands.turret.setTurretPosPID;
import frc.robot.nsubsystems.Turret;


public class backToStartingConfig extends SequentialCommandGroup {

  public backToStartingConfig(Turret turret, Joystick manipJoystick) {
    super(
      new setTurretPosPID(turret, -15, 0),
      new TurnOffLimelight(turret, manipJoystick)
    );
  }
}
