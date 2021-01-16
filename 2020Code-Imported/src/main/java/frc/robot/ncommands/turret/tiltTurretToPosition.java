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
  //declare required subsystem
  Turret turret;
  //variables used in calulating the power required
  double tiltTicks,positionError, desiredTicks;
  
  //required input of the subsystem and the desired location
  public tiltTurretToPosition(Turret subsystem, double _desiredTicks) {

    //Assign values to public locations
    turret = subsystem;
    desiredTicks = _desiredTicks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //find the current position
    tiltTicks = turret.getTalonPosition(turret.tiltTalon);

    //Find the error based off current and where it should be
    positionError = Math.abs(tiltTicks - desiredTicks);

    //Determin minimum power direction
    boolean runForward = tiltTicks < desiredTicks;
    double runModifier = runForward ? -1: 1;
    
    //variables for tuning
		double minPower = .3;
		double P;
		P = .52;
	
    //power based off minimum power and how far off it still is then limit it
    double power = (runModifier * minPower) + (positionError/90 * P);
    turret.limit(power, .7, -.7);

    //set the power
    turret.tiltTalon.set(ControlMode.PercentOutput, power);
		SmartDashboard.putNumber("Turret tilt error", positionError);
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
