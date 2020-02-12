/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.Turret;
import frc.robot.nsubsystems.TurretLimelight;

public class shootTurretDistance extends CommandBase {
  /**
   * Creates a new shootTurretDistance.
   */
  TurretLimelight limelight;
  Turret turret;
  double distToTarget;

  public shootTurretDistance(Turret _turret, TurretLimelight _limelight) 
  {
    turret = _turret;
    limelight = _limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    distToTarget = limelight.distToTarget();
    SmartDashboard.putString("Turret Command Running:", "Shoot Turret w/Distance");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    double topShooterActual = turret.getTopWheelRPM();
    double bottomShooterActual = turret.getBottomWheelRPM();

    double topShooterDesired = 4.787*Math.pow(distToTarget, 2) + 110.3889*distToTarget +1676.622;
    double bottomShooterDesired = topShooterDesired *1.25;

    double feederMax = 4500;
    double feederDesired = -.25*feederMax;
    double feederActual = turret.getFeederRPM();

    double topShooterError = topShooterDesired-topShooterActual;
    double bottomShooterError = bottomShooterDesired-bottomShooterActual;
    double feederError = feederDesired-feederActual;

    double topShooterPower = topShooterDesired+topShooterError;
    double bottomShooterPower = bottomShooterDesired+bottomShooterError;
    double feederPower = feederDesired+feederError;

    topShooterPower = topShooterPower/5600;
    bottomShooterPower = bottomShooterPower/5200;
    feederPower = feederPower/feederMax;

    double tP, bP, fP, tI, bI;

    tP = .99; tI = .4;
    bP = .968; bI = .4;
    fP = .892;

    double tErrorSum =+ topShooterPower *.02;
    double bErrorSum =+ bottomShooterPower * .02;

    double topShooterInput = (topShooterPower * tP) + (tErrorSum * tI);
    double bottomShooterInput = (bottomShooterPower * bP) + (bErrorSum * bI);
    double feederInput = feederPower*fP;

    double minError = 50;
    
    boolean isUpToSpeed = 
    (topShooterError <= minError && bottomShooterError <=minError);

    feederInput = isUpToSpeed ? feederInput : 0;
    
    turret.runBallFeeder(feederInput);
    turret.runShooterPower(topShooterInput, bottomShooterInput);

    SmartDashboard.putBoolean("is Shooter Up to Speed", isUpToSpeed);
    SmartDashboard.putNumber("Desired top RPM ", topShooterDesired);
    SmartDashboard.putNumber("Top Shooter Input", topShooterInput);
    SmartDashboard.putNumber("Bottom Shooter Input", bottomShooterInput);
    SmartDashboard.putNumber("Feeder Input", feederInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.runShooterPower(0, 0);
    turret.runBallFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
