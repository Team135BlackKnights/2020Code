/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.turret;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.*;
import frc.robot.util.ImprovedJoystick;

public class TurretTest extends CommandBase {
  /**
   * Creates a new TurretTest.
   */
  private final Turret turret; 
  public ImprovedJoystick _joystick;
  public TurretTest(Turret subsystem, Joystick joystick) {
    turret = subsystem;
    _joystick = new ImprovedJoystick(joystick);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    SmartDashboard.putString("Turret Command Running: ", "turret Temp");
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double manipYpower, manipZpower,topRPM, bottomRPM, feederPower;
    
    manipYpower = _joystick.getJoystickAxis(1);
    manipZpower = _joystick.getJoystickAxis(2);

    boolean isTriggerPressed;
    isTriggerPressed =  _joystick.getJoystickButtonValue(1);
    
    if(isTriggerPressed)
    {
      topRPM = 2200;
      feederPower = .6;
    }
    else 
    {
      topRPM = 0;
      feederPower = 0;
    }
    bottomRPM = topRPM*3/2;

    turret.aimTurret(manipZpower, manipYpower);
    turret.runShooterRPM(topRPM, bottomRPM);
    turret.runBallFeeder(feederPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopAllTurretMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
