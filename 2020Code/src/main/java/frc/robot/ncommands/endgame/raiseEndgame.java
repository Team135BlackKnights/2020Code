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

public class raiseEndgame extends CommandBase {
  
  private final Endgame endgame;
  private double _target;
  private double targetError;
  public raiseEndgame(Endgame subsystem, double target) 
  {
    endgame = subsystem;
    _target = target;
    addRequirements(endgame);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("Endgame command Running: ", "raise Endgame " + _target);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double currentLiftPos, kp, power, minPower, minDirection;

    currentLiftPos = endgame.getLiftRaiseEncoderPosition();
    targetError = _target-currentLiftPos;

    power = targetError/90;
    minPower = .15;
    minDirection = targetError > 0 ? 1:-1;
    kp = 1; 

    minPower = minPower * minDirection;

    power = (minPower + (power * kp));
    endgame.runLiftRaiseSpark(power);
    SmartDashboard.putNumber("Raise Endgame target Error ", targetError);
    SmartDashboard.putNumber("Raise Endgame power: ", power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endgame.runLiftRaiseSpark(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetError) <= 2;
  }
}
