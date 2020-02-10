/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.storage;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Storage;
import frc.robot.util.ImprovedJoystick;

public class conveyorTest extends CommandBase {
  
  private final Storage storage;
  private final ImprovedJoystick _joystick;
  public conveyorTest(Storage subsystem, Joystick joystick) {
    storage = subsystem;
    _joystick = new ImprovedJoystick(joystick);

  addRequirements(storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Storage Commmand Running: ", " runConveyor");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    storage.runConveyor(_joystick.getJoystickAxis(1));
    SmartDashboard.putNumber("conveyor test power", _joystick.getJoystickAxis(1));
    //endgame.runLiftRaiseSpark(-_joystick.getJoystickAxis(1)/3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storage.runConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
