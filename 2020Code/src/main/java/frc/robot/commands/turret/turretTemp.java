/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class turretTemp extends Command {
  public turretTemp() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    SmartDashboard.putString("Turret Command Running: ", "turret Temp");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {

    double manipYpower, manipZpower, manipXpower,shooterPower, shooterRPM;

    shooterPower= Robot.oi.returnManipSlider();
    manipYpower = Robot.oi.GetJoystickYValue(2);
    manipXpower = Robot.oi.GetJoystickXValue(2);
    manipZpower = Robot.oi.GetJoystickZValue(2);
    shooterRPM = shooterPower*4500;
    Robot.turret.aimTurret(manipZpower, manipYpower);

    boolean isTriggerPressed = Robot.oi.manipTrigger();
    boolean isThumbPressed = Robot.oi.getManipThumb();
    double magicBoi = .4211;

    if(isTriggerPressed)
    {
      shooterPower = -shooterPower;
    }
    else if(isThumbPressed)
    {
      Robot.turret.runShooterRPM(shooterRPM, -shooterRPM*magicBoi);
    }
    else if(isTriggerPressed && isThumbPressed)
    {
      shooterPower = -shooterPower;
    }
    else
    {
      shooterPower = shooterPower*4500;
    }
    double feederPower = .65;
    if(Robot.oi.getButtonOutPut(2, 3))
    {
      feederPower = -.65;
    }
    else{
      feederPower = .65;
    }
    double botShooterRPM = shooterRPM*magicBoi;

    Robot.turret.runShooterRPM(shooterRPM, botShooterRPM);
    Robot.turret.runBallFeeder(feederPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.turret.aimTurret(0, 0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
