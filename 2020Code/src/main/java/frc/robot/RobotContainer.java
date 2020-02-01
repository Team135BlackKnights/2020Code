
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.endgame.joystickEndgame;
import frc.robot.commands.turret.toggleLight;
import frc.robot.commands.turret.turretTemp;
import frc.robot.ncommands.color.rotateWheelOfFortune;
import frc.robot.ncommands.drive.DriveWithJoysticks;
import frc.robot.ncommands.drive.shiftGears;
import frc.robot.ncommands.drive.toggleCompressor;
import frc.robot.ncommands.endgame.raiseEndgame;
import frc.robot.ncommands.endgame.runEndgameWithJoystick;
import frc.robot.ncommands.endgame.runWinch;
import frc.robot.ncommands.intake.moveIntake;
import frc.robot.ncommands.intake.runConveyor;
import frc.robot.ncommands.intake.runRoller;
import frc.robot.ncommands.turret.TargetTurret;
import frc.robot.ncommands.turret.ToggleLight;
import frc.robot.ncommands.turret.TurretTest;
import frc.robot.nsubsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements RobotMap{
  // The robot's subsystems and commands are defined here...
  private final FalconDrive drive = new FalconDrive();
  private final Turret turret = new Turret();
  private final Intake intake = new Intake();
  private final Endgame endgame = new Endgame();
  private final ColorWheel colorWheel = new ColorWheel();
  private final TurretLimelight turretLimelight = TurretLimelight.getInstance();
  private final IntakeLimelight intakeLimelight = IntakeLimelight.getInstance();
  


  public static Joystick 
  leftJoystick = new Joystick(RobotMap.KOI.LEFT_JOYSTICK),
  rightJoystick = new Joystick(RobotMap.KOI.RIGHT_JOYSTICK),
  manipJoystick = new Joystick(RobotMap.KOI.MANIP_JOYSTICK);
  
  public final int 
  TOP_POV = 0 ,
  RIGHT_POV = 1, 
  BOTTOM_POV = 2,
  LEFT_POV = 3,
  TOP_RIGHT_POV = 4,
  BOTTOM_RIGHT_POV = 5,
  BOTTOM_LEFT_POV = 6, 
  TOP_LEFT_POV = 7,
  LEFT_JOYSTICK = RobotMap.KOI.LEFT_JOYSTICK,
  RIGHT_JOYSTICK = RobotMap.KOI.RIGHT_JOYSTICK,
  MANIP_JOYSTICK = RobotMap.KOI.MANIP_JOYSTICK;

public static Joystick[] joysticks = {leftJoystick, rightJoystick, manipJoystick};

public static JoystickButton 
	rightTrigger = new JoystickButton(rightJoystick, KOI.TRIGGER_BUTTON),
	rightThumb = new JoystickButton(rightJoystick, KOI.THUMB_BUTTON),
	rightButton3 = new JoystickButton(rightJoystick, KOI.HANDLE_BOTTOM_LEFT_BUTTON),
	

	leftTrigger = new JoystickButton(leftJoystick, KOI.TRIGGER_BUTTON),
	leftThumb = new JoystickButton(leftJoystick, KOI.THUMB_BUTTON),
	leftButton11 = new JoystickButton(leftJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),

	manipTrigger = new JoystickButton(manipJoystick, KOI.TRIGGER_BUTTON),
	manipThumb = new JoystickButton(manipJoystick, KOI.THUMB_BUTTON),
    manipButton3 = new JoystickButton(manipJoystick, KOI.HANDLE_BOTTOM_LEFT_BUTTON),
    manipButton4 = new JoystickButton(manipJoystick, KOI.HANDLE_BOTTOM_RIGHT_BUTTON),
	manipButton5 = new JoystickButton(manipJoystick, KOI.HANDLE_BOTTOM_RIGHT_BUTTON),
	manipButton7 = new JoystickButton(manipJoystick, KOI.BASE_TOP_LEFT_BUTTON),
	manipButton8 = new JoystickButton(manipJoystick, KOI.BASE_TOP_RIGHT_BUTTON),
	manipButton9 = new JoystickButton(manipJoystick, KOI.BASE_MIDDLE_LEFT_BUTTON),
	manipButton10 = new JoystickButton(manipJoystick, KOI.BASE_MIDDLE_RIGHT_BUTTON),
	manipButton11 = new JoystickButton(manipJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
	manipButton12 = new JoystickButton(manipJoystick, KOI.BASE_BOTTOM_RIGHT_BUTTON);

		


  public RobotContainer() {
      
    drive.setDefaultCommand(new DriveWithJoysticks(drive, leftJoystick, rightJoystick));
    intake.setDefaultCommand(new runConveyor(intake, manipJoystick));
    //turret.setDefaultCommand(new TargetTurret(turret, turretLimelight, manipJoystick));
    turret.setDefaultCommand(new TurretTest(turret, manipJoystick));

    // Configure the button bindings
    configureButtonBindings();
   
  }


  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() 
  {
    rightButton3.whenPressed(new ToggleLight(turret));
    leftThumb.whenPressed(new shiftGears(drive));
	leftButton11.toggleWhenPressed(new toggleCompressor(drive));

    manipTrigger.whileHeld(new runRoller(intake, .9));
    manipThumb.whileHeld(new runEndgameWithJoystick(endgame, manipJoystick));
    
    manipButton3.whenPressed(new rotateWheelOfFortune(colorWheel, .8));
    manipButton4.whileHeld(new runRoller(intake, -.8));
    manipButton5.whileHeld(new runWinch(endgame, .675));
    manipButton9.whenPressed(new rotateWheelOfFortune(colorWheel, 0));
    manipButton10.whenPressed(new raiseEndgame(endgame, 8));
    manipButton11.toggleWhenPressed(new moveIntake(intake));
    manipButton12.whenPressed(new raiseEndgame(endgame, 10));
    
  }

  public static boolean leftTrigger() {
    return leftTrigger.get();
  }
  
  public static boolean rightTrigger()
  {
    return rightTrigger.get();
  }
  
  public boolean manipTrigger()
  {
      return manipTrigger.get();
  }
  
  public static boolean leftThumb()
  {
      return leftThumb.get();
  }
  
  public static boolean rightThumb()
  {
      return rightThumb.get();
  }
  public boolean getManipThumb()
  {
      return manipThumb.get();
  }
  
  public boolean getManipButton7()
  {
      return manipButton7.get();
  }
  
  public boolean getManipButton8()
  {
      return manipButton8.get();
  }
  
  public boolean getButtonOutPut(int joystick, int buttonID)
  {
      return joysticks[joystick].getRawButton(buttonID);
  }
  
  
  public double getThrottle(Joystick joystick)
  {
      return joystick.getThrottle();
  }
  
  
  
  
  //returns value of the given joystick with a deadband applied
      private double DeadbandJoystickValue(double joystickValue) {
          return (Math.abs(joystickValue) < KOI.JOYSTICK_DEADBAND ? 0.0 : joystickValue);
      }
      public double GetJoystickYValue(int joystickNumber) {
          return DeadbandJoystickValue( -joysticks[joystickNumber].getY() );
      }
      public double GetJoystickXValue(int joystickNumber) {
          return DeadbandJoystickValue( -joysticks[joystickNumber].getX() );
      }
      public double GetJoystickZValue(int joystickNumber) {
          return DeadbandJoystickValue( -joysticks[joystickNumber].getZ() );
      }
  
      public double getPovAngle(int joystickID)
      {
          return joysticks[joystickID].getPOV();
          
      }
  
      public boolean isPovDirectionPressed(int joystickID, int povDirection)
      {
          double povValue = this.getPovAngle(joystickID);
          boolean povDirectionPressed = false;
          switch(povDirection) {
  
          case (TOP_POV):
          if (povValue >= 337.5 || (povValue <= 22.5 && povValue != -1)) {
              povDirectionPressed = true;
          }
          else {
              povDirectionPressed = false;
          }
          break;
  
          case (TOP_RIGHT_POV):
          if (povValue >= 22.5 && (povValue <= 67.5 )) {
              povDirectionPressed = true;
          }
          else {
              povDirectionPressed = false;
          }
          break;
      case (RIGHT_POV):
          if (povValue >= 67.5 && povValue <= 112.5) {
              povDirectionPressed = true;
          }
          else {
              povDirectionPressed = false;
          }
          break;
  
          case (BOTTOM_RIGHT_POV):
          if (povValue >= 112.5 && povValue <= 157.5) {
              povDirectionPressed = true;
          }
          else {
              povDirectionPressed = false;
          }
          break;
      case (BOTTOM_POV):
          if (povValue >= 157.5 && povValue <= 202.5) {
              povDirectionPressed = true;
          }
          else {
              povDirectionPressed = false;
          }
          break;
      case (BOTTOM_LEFT_POV):
          if (povValue >= 202.5 && povValue <= 247.5) {
              povDirectionPressed = true;
          }
          else {
              povDirectionPressed = false;
          }
          break;
      case (LEFT_POV):
          if (povValue >= 247.5 && povValue <= 337.5) {
              povDirectionPressed = true;
          }
          else {
              povDirectionPressed = false;
          }
          break;
          case (TOP_LEFT_POV):
          if (povValue >= 292.5 && povValue <= 292.5) {
              povDirectionPressed = true;
          }
          else {
              povDirectionPressed = false;
          }
          break;
      
      }
      return povDirectionPressed;
  
      }
  
      public Command getAutonomousCommand() {

        drive.resetEncoders();
        drive.resetHeading();
        drive.resetOdometry();
    
        TrajectoryConfig config = new TrajectoryConfig(3.97350993, 2);
    
        config.setKinematics(drive.getKinematics());
    
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          Arrays.asList(
            new Pose2d(), 
            new Pose2d(3, -2, Rotation2d.fromDegrees(0))
            ), 
          config
        );
    
        RamseteController disabledRamsete = new RamseteController() {
          @Override
          public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                 double angularVelocityRefRadiansPerSecond) {
            return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
          }
        };
    
      
      var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
      var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
      var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
      var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");
    
        RamseteCommand command = new RamseteCommand(
          trajectory,
          m_driveSubsystem::getPose,
          disabledRamsete,//new RamseteController(2.0, 0.7),
          m_driveSubsystem.getFeedForward(),
          m_driveSubsystem.getKinematics(),
          m_driveSubsystem::getWheelSpeeds,
          m_driveSubsystem.getLeftPIDController(),
          m_driveSubsystem.getRightPIDController(),
          (leftVolts, rightVolts) -> {
            m_driveSubsystem.set(leftVolts, rightVolts);
    
            m_leftMeasurement.setNumber(m_driveSubsystem.getFeedForward().calculate(m_driveSubsystem.getWheelSpeeds().leftMetersPerSecond));
            m_leftReference.setNumber(leftVolts);
    
            m_rightMeasurement.setNumber(m_driveSubsystem.getFeedForward().calculate(m_driveSubsystem.getWheelSpeeds().rightMetersPerSecond));
            m_rightReference.setNumber(-rightVolts);
        },//m_driveSubsystem::set,
          m_driveSubsystem
        );



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
  

}

