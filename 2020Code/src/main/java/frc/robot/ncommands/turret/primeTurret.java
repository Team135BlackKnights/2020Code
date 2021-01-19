/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Turret;

public class primeTurret extends CommandBase {
  /**
   * Creates a new primeTurret.
   */
  Turret turret; 

  public primeTurret(Turret _turret) 
  {
    turret = _turret;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double shooterDesired, shooterError, shooterActual, shooterInput, shooterFeedforward, kP, kF;

    shooterDesired = 1800;
    shooterActual = turret.getShooterVel();

    shooterError = (shooterDesired - shooterActual)/6000;

    shooterFeedforward = shooterDesired /turret.maxRPM;

    kP = 10000;
    kF = .625;

    shooterInput = kF * shooterFeedforward + kP * shooterError;

    turret.runShooter(shooterInput);
    turret.runIndexer(shooterInput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    turret.runShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
