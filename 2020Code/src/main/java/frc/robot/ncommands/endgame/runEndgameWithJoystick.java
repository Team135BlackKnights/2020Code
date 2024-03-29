/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.endgame;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Endgame;
import frc.robot.util.ImprovedJoystick;

public class runEndgameWithJoystick extends CommandBase {

  private final Endgame endgame;
  private final ImprovedJoystick _joystick;

  public runEndgameWithJoystick(Endgame subsystem, Joystick joystick) {
    endgame = subsystem;
    _joystick = new ImprovedJoystick(joystick);

    addRequirements(endgame);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Endgame command Running: ", "Run Endgame with Joystick");
    endgame.setShifterPos(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //sets power to the lift
    endgame.runLiftRaiseSpark(-_joystick.getJoystickAxis(1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Endgame Command Running: ", "Run Endgame with Joystick");
    endgame.runLiftRaiseSpark(0);
    endgame.setShifterPos(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
