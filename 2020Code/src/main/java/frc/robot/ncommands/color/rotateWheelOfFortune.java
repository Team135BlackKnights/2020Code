/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.color;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.ColorWheel;

public class rotateWheelOfFortune extends CommandBase {
  ColorWheel colorWheel;
  public boolean isFinished;
  public double manualSpinSpeed;
  /**
   * Creates a new rotateWheelOfFortune.
   */
  public rotateWheelOfFortune(ColorWheel subsystem, double manualSpin) {
    manualSpinSpeed = manualSpin;
    colorWheel = subsystem;
    addRequirements(colorWheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished=false;

    SmartDashboard.putString("Control Panel Command Running:", "rotate Wheel of Fortune");
    colorWheel.moveExtend(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (manualSpinSpeed != 0)//if = 0, then command is not manual control and skip this block
    {
      colorWheel.rotatorSpark.set(manualSpinSpeed);
      edu.wpi.first.wpilibj.Timer t = new edu.wpi.first.wpilibj.Timer();
      t.start();
      double starttime = t.get();
      while ((t.get() - starttime) < .25 )//runs motor for .25 seconds
      {}
      colorWheel.rotatorSpark.set(0);
      isFinished = true;
      return;
    }
    
    //Sets the desired color to the color given by the game
    String DesiredColor = colorWheel.gameColor();
    SmartDashboard.putString("FMS Readout", DesiredColor);
    //If the desired color isn't empty, rotate the wheel at 80% power until it is detected 
    if (DesiredColor != "No Color") {
      colorWheel.getToColor(DesiredColor, .25);
      if (colorWheel.atDesiredRoations){ //2If the wheel has been spun the desired amount, it is finished
        isFinished = true;
        colorWheel.stopControlPanel();
        }
    }
    else {
      //If the desired color is no color, rotate the wheel four times at 80% power
      colorWheel.rotateColorWheel(.8, 2.75);
      if (colorWheel.checkForColor() != colorWheel.desiredColor) {
        isFinished = true;
        colorWheel.stopControlPanel();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorWheel.stopControlPanel();
    colorWheel.rotatorSpark.set(0);
    SmartDashboard.putString("Control Panel Command Running:", "No command Running");
    colorWheel.atDesiredRoations=false;
    colorWheel.moveExtend(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
