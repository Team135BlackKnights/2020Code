
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.ncommands.color.rotateWheelOfFortune;
import frc.robot.ncommands.drive.*;
import frc.robot.ncommands.endgame.*;
import frc.robot.ncommands.storage.*;
import frc.robot.ncommands.intake.*;
import frc.robot.ncommands.turret.*;
import frc.robot.nsubsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer implements RobotMap {
  // The robot's subsystems and commands are defined here...
  private final Storage storage = new Storage();
  private final FalconDrive drive = new FalconDrive();
  private final Turret turret = new Turret();
  private final Intake intake = new Intake();
  private final Endgame endgame = new Endgame();
  private final ColorWheel colorWheel = new ColorWheel();
  private final TurretLimelight turretLimelight = TurretLimelight.getInstance();

  //Ramsete Variables
  private static final double kS = 0.358;
  private static final double kV = 3.02;
  private static final double kA = 0.249;
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(23));
  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);
  Trajectory trajectory;
  String trajectoryJSON;
  public RamseteCommand Steven;

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
      rightButton11 = new JoystickButton(rightJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
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
    drive.setDefaultCommand(new DriveWithJoysticks(drive, leftJoystick, rightJoystick));
    storage.setDefaultCommand(new runConveyor(storage, manipJoystick));
    turret.setDefaultCommand(new TargetTurret(turret, turretLimelight, manipJoystick));
    // turret.setDefaultCommand(new TurretTest(turret, manipJoystick));
    // Configure the button bindings
    initRamsete("paths/redRight.wpilib.json");
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    rightButton3.whenPressed(new ToggleLight(turret));
    rightButton11.whenPressed(Steven);
    leftThumb.whenPressed(new shiftGears(drive));
    leftButton7.whenPressed(new resetDriveEncoders(drive));
    leftButton8.whenPressed(new resetEndgameEncoders(endgame));
    leftButton9.whenPressed(new resetIntakeEncoders(intake));
    leftButton10.whenPressed(new resetStorageEncoders(storage));
    leftButton11.toggleWhenPressed(new toggleCompressor(drive));
    leftButton12.whenPressed(new resetTurretEncoder(turret));

    manipTrigger.whileHeld(new runTurretAndStorage(storage, turret, turretLimelight));
    manipThumb.whileHeld(new runEndgameWithJoystick(endgame, manipJoystick));
    //manipButton3.whenPressed(new rotateWheelOfFortune(colorWheel, .25));
  // manipButton3.whenPressed(new RotateTurretToAngle(turret, -90));
    
    manipButton4.whileHeld(new runRoller(intake, .4));
    manipButton3.whileHeld(new runWinch(endgame, .8));
    manipButton9.whenPressed(new rotateWheelOfFortune(colorWheel, 0));
    manipButton10.whenPressed(new raiseEndgame(endgame, 222));
    manipButton11.toggleWhenPressed(new moveIntake(intake));
    manipButton12.whenPressed(new raiseEndgame(endgame, 165));

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

  public void initRamsete(String trajectoryJSON) {
  drive.resetEncoders();
    drive.getAngle();
    // drive.resetOdometry();
    TrajectoryConfig config = new TrajectoryConfig(3.97350993, 2);
    config.setKinematics(getKinematics());
    // final String trajectoryJSON = "paths/YourPath.wpilib.json";
    try {
      final Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (final IOException ex) {
      SmartDashboard.putString("Unable to open trajectory: ", "");
    }

    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
          double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
    };


    var leftController = new PIDController(.6, 0, 0);
    var rightController = new PIDController(.6, 0, 0);
    Steven = new RamseteCommand(trajectory, drive::getPose, disabledRamsete, getFeedForward(),
        getKinematics(), drive::getWheelSpeeds, leftController, rightController, (leftVolts, rightVolts) -> {
          drive.tankVolts(leftVolts, rightVolts);
        }, // m_driveSubsystem::set,
        drive);
  }


  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedForward;
  }

  public Command getAutonomousCommand() {

    return null;
  }
}
