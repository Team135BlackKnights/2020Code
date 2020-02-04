/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.*;

public class runConveyorSetPos extends CommandBase {
  /**
   * Creates a new runConveyorSetPos.
   */
  private final Storage storage;
  private final double _targetPos;
  private boolean isFinished = false;
  public runConveyorSetPos(Storage subsystem, double targetPos) {
    storage = subsystem;
    this._targetPos = targetPos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Intake Command Running: ", " RunConveyorSetPos");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentConveyPos = storage.getConveyorRotations();
    if((_targetPos)!=currentConveyPos)
    {
      storage.runConveyor(.65);
      isFinished = false;
    }
    else 
    {
      storage.runConveyor(0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
