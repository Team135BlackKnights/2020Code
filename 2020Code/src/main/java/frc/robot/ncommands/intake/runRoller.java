/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.Intake;

public class runRoller extends CommandBase {

  private final Intake intake;
  private  double _RPM;
  public boolean isWaiting;
  public double usedPower;

  public runRoller(Intake subsystem, double RPM, boolean waitForTurret) {
    intake = subsystem;
    _RPM = RPM >1000 ? RPM : RPM *10000;
    isWaiting = waitForTurret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double actualRPM = intake.getRollerRPM();
    double error = _RPM-actualRPM;
    
   double input = (_RPM+error)/10000;

    if(isWaiting)
    {
      if(RobotContainer.activeBallCount >=3)
      {
        input = 0;
      }
      else 
      {
        input = (_RPM+error)/10000;
      }
    }
    
    intake.runRoller(input);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
