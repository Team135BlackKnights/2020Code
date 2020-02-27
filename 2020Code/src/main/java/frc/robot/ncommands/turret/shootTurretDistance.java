/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.Turret;
import frc.robot.nsubsystems.TurretLimelight;

public class shootTurretDistance extends CommandBase {
  /**
   * Creates a new shootTurretDistance.
   */
  TurretLimelight limelight;
  Turret turret;
  double distToTarget;
  boolean isAuton;
  
  double tErrorSum, bErrorSum;
  public shootTurretDistance(Turret _turret, TurretLimelight _limelight, boolean _isAuton) 
  {
    turret = _turret;
    limelight = _limelight;
    isAuton = _isAuton;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    distToTarget = limelight.distToTarget();
    RobotContainer.limelight.initLimelight(0, 0);
    tErrorSum = 0;
    bErrorSum = 0; 
    SmartDashboard.putString("Turret Command Running: ", "Shoot Turret w/Distance");
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    double topShooterActual = turret.getTopWheelRPM();
    double bottomShooterActual = turret.getBottomWheelRPM();
    double topShooterMax = 5100;
    double bottomShooterMax = 5000;

   // double topShooterDesired = 4.787*Math.pow(distToTarget, 2) + 110.3889*distToTarget +2076.622;
    double steve = 150;
    double topShooterDesired = 5.5113*Math.pow(distToTarget, 3) - 72.1904*(Math.pow(distToTarget, 2)) 
                                + 428.5246*distToTarget +1346.0346 ;
    double bottomShooterDesired = topShooterDesired *1.25;

    double feederMax = 5000;
    double feederDesired = -.35*feederMax;
    double feederActual = turret.getFeederRPM();

    double topShooterError = (topShooterDesired-topShooterActual)/topShooterMax;
    double bottomShooterError = (bottomShooterDesired-bottomShooterActual)/bottomShooterMax;
    double feederError = (feederDesired-feederActual)/5250;

    double tP, bP, fP, tI, bI, tF, bF, fF;
    tF = .85;
    bF = .85;
    tP = .5; tI = 2.5;
    bP = .94; bI = 2;
    fF =.892;
    fP = 0;

    //tErrorSum =+ topShooterError/5600 *.02;
    tErrorSum = tErrorSum + (tI*topShooterError * .02);
    bErrorSum = bErrorSum + (bI*bottomShooterError * .02);

    tErrorSum = turret.limit(tErrorSum, .1, -.1);
    bErrorSum = turret.limit(bErrorSum, .1, -.1);

    SmartDashboard.putNumber("bottom error sum ", bErrorSum);
    SmartDashboard.putNumber("top Error sum ", tErrorSum);

    double topShooterInput, bottomShooterInput, feederInput;

    double minError = .15;
    
    turret.isShooterUpToSpeed = (topShooterError <= minError && bottomShooterError <=minError);
    double topShooterFeedForward = topShooterDesired/topShooterMax;
    double bottomShooterFeedForward = bottomShooterDesired/bottomShooterMax;
    double feederFeedForward = feederDesired/feederMax;
        topShooterInput = (topShooterFeedForward*tF) + (topShooterError * tP) + (tErrorSum);
        bottomShooterInput = (bottomShooterFeedForward*bF) + (bottomShooterError * bP) + (bErrorSum);
        feederInput = feederFeedForward * fF + feederError*fP;
        feederInput = turret.isShooterUpToSpeed ? feederInput : 0;
    
    turret.runBallFeeder(feederInput);
    turret.runShooterPower(topShooterInput, bottomShooterInput);
    SmartDashboard.putNumber("Top shooter input", topShooterInput);
    SmartDashboard.putNumber("Top shooter RPm", topShooterActual);
    SmartDashboard.putNumber("feeder RPM", feederActual);

    SmartDashboard.putNumber("Desired top RPM ", topShooterDesired);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    SmartDashboard.putString("Turret Command Running: ", "No Command Running");
    turret.runShooterPower(0, 0);
    turret.runBallFeeder(0);
    turret.isShooterUpToSpeed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
