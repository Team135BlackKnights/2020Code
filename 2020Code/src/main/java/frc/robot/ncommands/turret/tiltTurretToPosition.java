/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Turret;

public class tiltTurretToPosition extends CommandBase {
  Turret turret;
  double tiltPosition, tiltTicks,positionError, desiredTicks;
  /**
   * Creates a new tiltTurretToAngle.
   */
  public tiltTurretToPosition(Turret subsystem, double _desiredTicks) {
    turret = subsystem;
    desiredTicks = _desiredTicks;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tiltTicks = turret.getTalonPosition(turret.tiltTalon);
    tiltPosition = turret.ticksToRotations(turret.getTalonPosition(turret.tiltTalon));

    positionError = Math.abs(tiltTicks - desiredTicks);
    boolean runForward = tiltTicks < desiredTicks;
		double runModifier = runForward ? -1: 1;
		double minPower = .3;
		double P, I, D;
		P = .52;
		I = 0; 
		D = 0;
  

    double power = (runModifier * minPower) + (positionError/90 * P);
    
    turret.limit(power, .7, -.7);

    turret.tiltTalon.set(ControlMode.PercentOutput, power);
		SmartDashboard.putNumber("angle error", positionError);

  
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(positionError) <= 2);
  }
}
