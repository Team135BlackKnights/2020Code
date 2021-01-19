package frc.robot.ncommands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nsubsystems.*;
import frc.robot.util.ImprovedJoystick;

public class DriveWithJoysticks extends CommandBase {
  private final FalconDrive drive;

  ImprovedJoystick _leftJoystick, _rightJoystick;

  public DriveWithJoysticks(FalconDrive subsystem, Joystick leftJoystick, Joystick rightJoystick) {
    drive = subsystem;
    addRequirements(drive);
    _leftJoystick = new ImprovedJoystick(leftJoystick);
    _rightJoystick = new ImprovedJoystick(rightJoystick);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Drive Command Running: ", "Drive with Joysticks");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double lateralPower, rotationPower;
    boolean isReversed = false, isHalfPower = true;

    // Get values of buttons/joystick
    lateralPower = _rightJoystick.getJoystickAxis(1) * _rightJoystick.getJoystickSlider();
    rotationPower = _leftJoystick.getJoystickAxis(2) * _leftJoystick.getJoystickSlider();
    isHalfPower = _rightJoystick.getJoystickButtonValue(2);
    isReversed = (_rightJoystick.getJoystickButtonValue(1));

    SmartDashboard.putNumber("Right Slider", _rightJoystick.getJoystickSlider());
    SmartDashboard.putNumber("Left Slider", _leftJoystick.getJoystickSlider());

    // Decrease power if button is held
    if (isHalfPower) {
      lateralPower = lateralPower * .75;
      rotationPower = rotationPower * .5;
    }
    // reverse if reverse is held down
    lateralPower = isReversed ? lateralPower : -lateralPower;

    // Run motor
    drive.ArcadeDrive(lateralPower, rotationPower * .85);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
