/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.storage;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.nsubsystems.*;
import frc.robot.util.ImprovedJoystick;
import frc.robot.util.MotorControl;

public class runConveyor extends CommandBase {

  private final Storage storage;
  private ImprovedJoystick _joystick;

  public runConveyor(Storage subsystem, Joystick joystick) {
    storage = subsystem;
    _joystick = new ImprovedJoystick(joystick);
    addRequirements(storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Storage Command Running: ", " runConveyor");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentConveyPos = MotorControl.getSparkEncoderPosition(storage.conveyorEncoder);

    // Power modifier to overcome friction
    double powerMod = RobotContainer.activeBallCount * .075;
    powerMod = RobotContainer.activeBallCount >= 4 ? powerMod : 0;

    if (storage.isBallAtTripSwitch()) {
      MotorControl.resetSparkEncoder(storage.conveyorEncoder);
    }
    double desiredPosition = -30;
    double conveyorError = desiredPosition - currentConveyPos;
    double conveyorPower = 0;
    boolean isButton7, isButton8;

    isButton7 = _joystick.getJoystickButtonValue(7);
    isButton8 = _joystick.getJoystickButtonValue(8);

    // For intaking powercells
    
    
    if (currentConveyPos > (desiredPosition) && !isButton7 && !isButton8) {
      //conveyorPower = (conveyorError * .16) - powerMod;

      //conveyorPower = conveyorError * (1 / Math.abs(desiredPosition));
      conveyorPower = -1;
      RobotContainer.intake.runRoller(0);
      // RobotContainer.turret.runBallFeeder(0);
      SmartDashboard.putString("Conveyor Override: ", "Sensor Control");
    } else if (isButton7) {
      conveyorPower = 1;
      MotorControl.resetSparkEncoder(storage.conveyorEncoder);
      RobotContainer.intake.runRoller(-.3);
      // RobotContainer.turret.runBallFeeder(.2);
      if (RobotContainer.intake.isRollerLowered()) {
        RobotContainer.intake.raiseLower(false);
      }
      SmartDashboard.putString("Conveyor Override: ", "Conveyor Going Up");
    }

    // Manual ball eject
    else if (isButton8) {
      conveyorPower = -1;
      RobotContainer.intake.runRoller(.3);
      MotorControl.resetSparkEncoder(storage.conveyorEncoder);
      SmartDashboard.putString("Conveyor Override: ", "Conveyor Going Down");
    }

    else {
      conveyorPower = 0;
      SmartDashboard.putString("Conveyor Override: ", "Sensor Control");
    }

    
    SmartDashboard.putNumber("run storage conveyor power", conveyorPower);
    storage.runConveyor(conveyorPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storage.runConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putString("Storage Command Running: ", "No Command Running");
    return false;
  }
}
