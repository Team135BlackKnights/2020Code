/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.endgame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Endgame;

public class runWinch extends CommandBase {
  /**
   * Creates a new runWinch.
   */
  private final Endgame endgame; 
  private double _power;
  public runWinch(Endgame subsystem, double power) 
  {
    endgame = subsystem;
    _power = power;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    SmartDashboard.putString("Endgame command Running: ", "runWinch" + _power);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endgame.runWinchSpark(_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endgame.runWinchSpark(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
