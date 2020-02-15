/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.*;
import frc.robot.util.MovingAverage;

public class ShootTurret extends CommandBase {

  private final Turret turret;
  private final TurretLimelight limelight;
  private final double topShooterDesired, bottomShooterDesired;
  private boolean isFinished;
  private double previousFeederError = 0;

  public ShootTurret(Turret subsystem, TurretLimelight _limelight,  double topShooterRPM, double bottomShooterRPM) {
    turret = subsystem;
    limelight = _limelight;
    topShooterDesired = topShooterRPM;
    bottomShooterDesired = bottomShooterRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("Turret Command Running:", "Shoot Turret ");
    SmartDashboard.putNumber("Top Desired", topShooterDesired);
    SmartDashboard.putNumber("Bottom Desired", bottomShooterDesired);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feederMax = 4500;

    double feederDesired = -.25 * feederMax;

    double topShooterActual = turret.getTopWheelRPM();
    double bottomShooterActual = turret.getBottomWheelRPM();
    double feederActual = turret.getFeederRPM();

    double distanceMod = limelight.distToTarget();
    double perferedDistance = 4;
    double usedDistance = distanceMod-perferedDistance;
    
    MovingAverage distance = new MovingAverage(40);

    double modifier = distance.process((float)usedDistance);

    double minError = 50;
    
    double tsd = topShooterDesired;
    double bsd = bottomShooterDesired;

   // tsd = topShooterDesired + tAddOn;
   // bsd = bottomShooterDesired + bAddOn;

    
    SmartDashboard.putNumber("Shooter distance Mod", modifier);
    SmartDashboard.putNumber("new Top Desired", tsd);
    SmartDashboard.putNumber("new Bottom Desired", bsd);

    double topShooterError = tsd-topShooterActual;
    double bottomShooterError = bsd-bottomShooterActual;
    double feederError = feederDesired-feederActual;
    boolean isUpToSpeed = 
    (topShooterError <= minError && bottomShooterError <=minError);

    double topPower = tsd + topShooterError;
    double bottomPower = bsd + bottomShooterError;
    double feederPower = feederDesired + feederError;

    topPower = topPower / 5600;
    bottomPower = bottomPower / 5200;
    feederPower = feederPower / feederMax;

    double tP, bP, fP, tI, bI, fD; 

    tP = .99; tI = .4;
    bP = .968; bI = .4;
    fP = .892; fD = 0;

    double tErrorSum =+ topPower * .02;
    double bErrorSum =+ bottomPower * .02;
    
    double feederDerivative = (feederPower-previousFeederError)/.02;


    double topInput = topPower * tP + tErrorSum * tI;
    double bottomInput = bottomPower * bP + bErrorSum * bI;;
    double feederInput = feederPower * fP + feederDerivative * fD;

    turret.runShooterPower(topInput, bottomInput);
    if(isUpToSpeed)
    {
      turret.runBallFeeder(feederInput);
    }
    else 
    {
      turret.runBallFeeder(0);
    }
    SmartDashboard.putBoolean("isShooterUpToSpeed", isUpToSpeed);
    SmartDashboard.putNumber("Feeder Power", feederInput);
    previousFeederError = feederPower;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.runShooterPower(0, 0);
    turret.runBallFeeder(0);
    SmartDashboard.putString("Turret Command Running:", "No Command Running");
    SmartDashboard.putString("Shoot Turret: ", "Has Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
