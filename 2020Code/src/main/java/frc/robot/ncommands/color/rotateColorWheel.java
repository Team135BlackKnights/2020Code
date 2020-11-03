/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.color;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.ColorWheel;

public class rotateColorWheel extends CommandBase {
  public ColorWheel wheel;
  double input, rotationPower, initTime, uhOhTime, desiredRotations, colorChanges;
  public boolean isFinished;
  public String desiredColor;
  public boolean finishedRotation;

  public rotateColorWheel(ColorWheel _wheel, double _input) {
    wheel = _wheel;
    input = _input;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    rotationPower = .5;
    uhOhTime = 5;
    initTime = Timer.getMatchTime();
    desiredColor = wheel.gameColor();
    wheel.colorChanges = 0;
    desiredRotations = 0;

    SmartDashboard.putString("Control Panel Command Running: ", "Rotate Color Wheel " + input);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wheel.countColor();
    String currentColor = wheel.currentColor;
    double currentRotations = wheel.colorChanges / 8;

    desiredRotations = desiredColor == "No Color" ? 4 : 0;
    double input = 0; // Value for input that isnt 0 because the if will never run
    boolean isDriving = Math.abs(RobotContainer.drive.getLinearMps()) > .2;

    //will never run due to input being set to 0
    if (input != 0) {
      if (Timer.getMatchTime() - initTime > uhOhTime) {
        wheel.moveColorWheel(input);
        isFinished = isDriving ? true : false;
      } else if (Timer.getMatchTime() - initTime < uhOhTime) {
        isFinished = true;
      }
    }
    //Control over testing if the command can end
    if ((desiredRotations == 0) && currentColor != desiredColor) {
      input = rotationPower;
      isFinished = isDriving ? true : false;
    } else if (desiredRotations > currentRotations) {
      input = rotationPower;
      isFinished = isDriving ? true : false;
    } else {
      input = 0;
      isFinished = true;
    }

    wheel.moveColorWheel(input);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wheel.stopControlPanel();
    SmartDashboard.putString("Control Panel Command Running: ", "No Command Running");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
