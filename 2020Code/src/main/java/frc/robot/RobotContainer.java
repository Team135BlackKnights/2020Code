
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.ncommands.color.*;
import frc.robot.ncommands.drive.*;
import frc.robot.ncommands.endgame.*;
import frc.robot.ncommands.storage.*;
import frc.robot.ncommands.intake.*;
import frc.robot.ncommands.turret.*;
import frc.robot.nsubsystems.*;
import frc.robot.ncommands.auton.*;
import frc.robot.ncommands.auton.parallels.leaveStartingConfig;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer implements RobotMap {
  // The robot's subsystems and commands are defined here...
  public static final FalconDrive drive = new FalconDrive();
  public static final Turret turret = new Turret();
  public static final Storage storage = new Storage();
  public static final Intake intake = new Intake();
  public static final Endgame endgame = new Endgame();
  public static final ColorWheel colorWheel = null;//new ColorWheel();
  public static final TurretLimelight limelight = TurretLimelight.getInstance();
  public static final Rioduino arduino = new Rioduino();
  public final autoLine autoLineCommand = new autoLine(drive, intake, turret);
  public final rightSideAuto rightSideAutoCommand = new rightSideAuto(drive, intake, turret, limelight, storage, true);
  public final autoLinePlus autoLinePlusCommand = new autoLinePlus(drive, intake, turret, limelight, storage);
  public final middleAuto middleAutoCommand = new middleAuto(drive, turret, limelight, intake, storage);
  public static int activeBallCount = 3;


  public static Joystick leftJoystick = new Joystick(RobotMap.KOI.LEFT_JOYSTICK),
      rightJoystick = new Joystick(RobotMap.KOI.RIGHT_JOYSTICK),
      manipJoystick = new Joystick(RobotMap.KOI.MANIP_JOYSTICK);

  public final int TOP_POV = 0, RIGHT_POV = 1, BOTTOM_POV = 2, LEFT_POV = 3, TOP_RIGHT_POV = 4, BOTTOM_RIGHT_POV = 5,
      BOTTOM_LEFT_POV = 6, TOP_LEFT_POV = 7, LEFT_JOYSTICK = RobotMap.KOI.LEFT_JOYSTICK,
      RIGHT_JOYSTICK = RobotMap.KOI.RIGHT_JOYSTICK, MANIP_JOYSTICK = RobotMap.KOI.MANIP_JOYSTICK;

  public static Joystick[] joysticks = { leftJoystick, rightJoystick, manipJoystick };

  public static JoystickButton rightTrigger = new JoystickButton(rightJoystick, KOI.TRIGGER_BUTTON),
      rightThumb = new JoystickButton(rightJoystick, KOI.THUMB_BUTTON),
      rightButton3 = new JoystickButton(rightJoystick, KOI.HANDLE_BOTTOM_LEFT_BUTTON),
      rightButton10 = new JoystickButton(rightJoystick , KOI.BASE_MIDDLE_RIGHT_BUTTON),
      rightButton11 = new JoystickButton(rightJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
      rightButton12 = new JoystickButton(rightJoystick, KOI.BASE_BOTTOM_RIGHT_BUTTON),
      leftTrigger = new JoystickButton(leftJoystick, KOI.TRIGGER_BUTTON),
      leftThumb = new JoystickButton(leftJoystick, KOI.THUMB_BUTTON),

      leftButton7 = new JoystickButton(leftJoystick, KOI.BASE_TOP_LEFT_BUTTON),
      leftButton8 = new JoystickButton(leftJoystick, KOI.BASE_TOP_RIGHT_BUTTON),
      leftButton9 = new JoystickButton(leftJoystick, KOI.BASE_MIDDLE_LEFT_BUTTON),
      leftButton10 = new JoystickButton(leftJoystick, KOI.BASE_MIDDLE_RIGHT_BUTTON),
      leftButton11 = new JoystickButton(leftJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
      leftButton12 = new JoystickButton(leftJoystick, KOI.BASE_BOTTOM_RIGHT_BUTTON),

      manipTrigger = new JoystickButton(manipJoystick, KOI.TRIGGER_BUTTON),
      manipThumb = new JoystickButton(manipJoystick, KOI.THUMB_BUTTON),
      manipButton3 = new JoystickButton(manipJoystick, KOI.HANDLE_BOTTOM_LEFT_BUTTON),
      manipButton4 = new JoystickButton(manipJoystick, KOI.HANDLE_BOTTOM_RIGHT_BUTTON),
      manipButton5 = new JoystickButton(manipJoystick, KOI.HANDLE_TOP_LEFT_BUTTON),
      manipButton7 = new JoystickButton(manipJoystick, KOI.BASE_TOP_LEFT_BUTTON),
      manipButton8 = new JoystickButton(manipJoystick, KOI.BASE_TOP_RIGHT_BUTTON),
      manipButton9 = new JoystickButton(manipJoystick, KOI.BASE_MIDDLE_LEFT_BUTTON),
      manipButton10 = new JoystickButton(manipJoystick, KOI.BASE_MIDDLE_RIGHT_BUTTON),
      manipButton11 = new JoystickButton(manipJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
      manipButton12 = new JoystickButton(manipJoystick, KOI.BASE_BOTTOM_RIGHT_BUTTON);

  public RobotContainer() {
    drive.setDefaultCommand(new driveWithJoysticks(drive, leftJoystick, rightJoystick));

     storage.setDefaultCommand(new runConveyor(storage, manipJoystick));
     turret.setDefaultCommand(new targetTurret(turret, limelight, manipJoystick));
    // turret.setDefaultCommand(new TurretTest(turret, manipJoystick));
    //turret.setDefaultCommand(new michiganTurretTeleop(turret, limelight, storage, manipJoystick));
    
    // Configure the button bindings
    //limelight.initLimelight(1, 1);
    
    
    configureButtonBindings();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    rightButton3.whenPressed(new toggleLight(turret));
    rightButton10.whenPressed(new resetGyro(drive));
    
    rightButton11.whenPressed(new leaveStartingConfig(intake, turret));
    //rightButton11.whenPressed(new rightSideAuto(drive, intake, turret, limelight, storage, false));
    
    leftTrigger.whenPressed(new shiftGears(drive));
    leftButton7.whenPressed(new resetDriveEncoders(drive));
    leftButton8.whenPressed(new resetEndgameEncoders(endgame));
    leftButton9.whenPressed(new resetIntakeEncoders(intake));
    leftButton10.whenPressed(new resetStorageEncoders(storage));
    leftButton11.toggleWhenPressed(new toggleCompressor(drive));
    leftButton12.whenPressed(new resetTurretEncoder(turret));

    manipTrigger.whileHeld(new runTurretAndStorage(storage, turret, limelight));
    manipThumb.whileHeld(new runEndgameWithJoystick(endgame, manipJoystick));
    manipButton3.whileHeld(new runRoller(intake,  2400 , false));
    manipButton5.toggleWhenPressed(new moveIntake(intake));
    //manipButton9.whenPressed(new rotateColorWheel(colorWheel, 0));
    manipButton10.whenPressed(new raiseEndgame(endgame, 165));
    manipButton11.whileHeld(new runWinch(endgame, -.85));
    manipButton12.whenPressed(new raiseEndgame(endgame, 15));
  }


  public static boolean leftTrigger() {
    return leftTrigger.get();
  }

  public static boolean rightTrigger() {
    return rightTrigger.get();
  }

  public boolean manipTrigger() {
    return manipTrigger.get();
  }

  public static boolean leftThumb() {
    return leftThumb.get();
  }

  public static boolean rightThumb() {
    return rightThumb.get();
  }

  public boolean getManipThumb() {
    return manipThumb.get();
  }


  public boolean getManipButton7() {
    return manipButton7.get();
  }

  public boolean getManipButton8() {
    return manipButton8.get();
  }

  public boolean getButtonOutPut(int joystick, int buttonID) {
    return joysticks[joystick].getRawButton(buttonID);
  }

  public double getThrottle(Joystick joystick) {
    return joystick.getThrottle();
  }

  // returns value of the given joystick with a deadband applied
  private double DeadbandJoystickValue(double joystickValue) {
    return (Math.abs(joystickValue) < KOI.JOYSTICK_DEADBAND ? 0.0 : joystickValue);
  }

  public double GetJoystickYValue(int joystickNumber) {
    return DeadbandJoystickValue(-joysticks[joystickNumber].getY());
  }

  public double GetJoystickXValue(int joystickNumber) {
    return DeadbandJoystickValue(-joysticks[joystickNumber].getX());
  }

  public double GetJoystickZValue(int joystickNumber) {
    return DeadbandJoystickValue(-joysticks[joystickNumber].getZ());
  }

  public double getPovAngle(int joystickID) {
    return joysticks[joystickID].getPOV();

  }

  public boolean isPovDirectionPressed(int joystickID, int povDirection) {
    double povValue = this.getPovAngle(joystickID);
    boolean povDirectionPressed = false;
    switch (povDirection) {

    case (TOP_POV):
      if (povValue >= 337.5 || (povValue <= 22.5 && povValue != -1)) {
        povDirectionPressed = true;
      } else {
        povDirectionPressed = false;
      }
      break;

    case (TOP_RIGHT_POV):
      if (povValue >= 22.5 && (povValue <= 67.5)) {
        povDirectionPressed = true;
      } else {
        povDirectionPressed = false;
      }
      break;
    case (RIGHT_POV):
      if (povValue >= 67.5 && povValue <= 112.5) {
        povDirectionPressed = true;
      } else {
        povDirectionPressed = false;
      }
      break;

    case (BOTTOM_RIGHT_POV):
      if (povValue >= 112.5 && povValue <= 157.5) {
        povDirectionPressed = true;
      } else {
        povDirectionPressed = false;
      }
      break;
    case (BOTTOM_POV):
      if (povValue >= 157.5 && povValue <= 202.5) {
        povDirectionPressed = true;
      } else {
        povDirectionPressed = false;
      }
      break;
    case (BOTTOM_LEFT_POV):
      if (povValue >= 202.5 && povValue <= 247.5) {
        povDirectionPressed = true;
      } else {
        povDirectionPressed = false;
      }
      break;
    case (LEFT_POV):
      if (povValue >= 247.5 && povValue <= 337.5) {
        povDirectionPressed = true;
      } else {
        povDirectionPressed = false;
      }
      break;
    case (TOP_LEFT_POV):
      if (povValue >= 292.5 && povValue <= 292.5) {
        povDirectionPressed = true;
      } else {
        povDirectionPressed = false;
      }
      break;

    }
    return povDirectionPressed;

  }

  public void initLimelight(int ledMode, int pipeline)
  {
    limelight.initLimelight(ledMode, pipeline);
  }

  public Command getAutoLinePlus()
  {
    return autoLinePlusCommand;
  }

  public Command getMiddleAuto()
  {
    return middleAutoCommand;

  }

  public Command getRightSideAuto()
  {
    return rightSideAutoCommand;

  }

  public Command getAutoLine()
  {
    return autoLineCommand;
  }
}
