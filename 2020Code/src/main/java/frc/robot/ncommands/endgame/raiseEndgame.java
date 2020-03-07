/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.endgame;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Endgame;
import frc.robot.util.MotorControl;

public class raiseEndgame extends CommandBase {

  private final Endgame endgame;
  private double _target;
  private double targetError;

  public raiseEndgame(Endgame subsystem, double target) {
    endgame = subsystem;
    _target = target;
    addRequirements(endgame);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("Endgame Command Running: ", "raise Endgame " + _target);
    if (_target < MotorControl.getSparkEncoderPosition(endgame.liftRaiseEncoder)) {
      endgame.setLiftBrakeMode(IdleMode.kBrake);
    }
    endgame.setShifterPos(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentLiftPos, kp, power, minPower, minDirection;

    currentLiftPos = MotorControl.getSparkEncoderPosition(endgame.liftRaiseEncoder);
    targetError = _target - currentLiftPos;

    power = targetError / 90;
    minPower = .15;
    minDirection = targetError > 0 ? 1 : -1;
    kp = 3.23;

    minPower = minPower * minDirection;

    power = (power * kp) + minPower;
    power = MotorControl.limit(power, .75, -.75);
    endgame.runLiftRaiseSpark(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Endgame Command Running: ", "No Command Running");
    endgame.runLiftRaiseSpark(0);
    endgame.setShifterPos(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetError) <= .5;
  }
}
