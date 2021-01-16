/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Turret;
import frc.robot.util.ImprovedJoystick;

public class TurnOffLimelight extends CommandBase {
  ImprovedJoystick joystick;
  Turret turret;
 
  public TurnOffLimelight(Turret _turret, Joystick _joystick) {
    joystick = new ImprovedJoystick(_joystick);
    turret = _turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.initLimelight(1,1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.initLimelight(0,0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(joystick.getJoystickAxis(0)) > .2 || Math.abs(joystick.getJoystickAxis(1)) > .2 || Math.abs(joystick.getJoystickAxis(2)) > .2;
  }
}
