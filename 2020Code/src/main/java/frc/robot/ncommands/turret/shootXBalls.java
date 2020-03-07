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

public class shootXBalls extends CommandBase {
  /**
   * Creates a new shootXBalls.
   */
  Turret turret;
  int ballsToShoot, ballsShot, initialBallsShot;
  boolean isFinished;
  double errorSum;

  public shootXBalls(Turret _turret, int _ballsToShoot) {
    turret = _turret;
    ballsToShoot = _ballsToShoot;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    turret.initLimelight(1, 0);
    initialBallsShot = turret.ballsShot;
    errorSum = 0;
    SmartDashboard.putString("New Turret Command Running: ", "set Turret To Pos");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ballsShot = turret.ballsShot - initialBallsShot;
    isFinished = ballsShot >= ballsToShoot;
    boolean readyForBall;

    double desiredRPM, actualRPM, rpmError, rpmFeedForward, kF, kP, kI, shooterInput;

    desiredRPM = 3500; // TODO change later
    actualRPM = turret.getShooterVel();
    rpmError = desiredRPM - actualRPM;

    rpmFeedForward = desiredRPM / turret.maxRPM;

    readyForBall = (Math.abs(rpmError) <= 150);
    kF = 1;
    kP = 0;
    kI = 0;

    errorSum = errorSum + (kI * rpmError);

    shooterInput = kF * rpmFeedForward + kP * rpmError + kI * errorSum;

    turret.runShooter(shooterInput);
    if (readyForBall) {
      turret.runIndexer(.5);
      RobotContainer.storage.runConveyor(.3);
    } else {
      turret.runIndexer(0);
      RobotContainer.storage.runConveyor(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopAllMotors();
    RobotContainer.storage.runConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
