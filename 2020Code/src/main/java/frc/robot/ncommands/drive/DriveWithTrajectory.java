/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ncommands.drive;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.nsubsystems.FalconDrive;

public class DriveWithTrajectory extends CommandBase {
  FalconDrive drive;
  private static final double distBetweenWheelsInches = 23;//26.84603809585759;
  private static final double gearRatio = 1 / 13.85;
  private static final double wheelDiameterInches = 6.375;//18;
  private static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
  private static final double encoderTicksPerRev = 2048;

  private static final double kS = 0.358;
  private static final double kV = 3.02;
  private static final double kA = 0.249;
  private static final double kP = 0.0;//0.0405;
  private static final double kD = 0.0;
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(distBetweenWheelsInches));
  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);
  Trajectory trajectory;
  final String trajectoryJSON;
  


  public DriveWithTrajectory(FalconDrive subsystem, String filePath) {
    drive = subsystem;
    trajectoryJSON = filePath;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {runRamsete();}

  public Command runRamsete() {
    drive.resetEncoders();
    drive.getAngle();
    //drive.resetOdometry();
    
    TrajectoryConfig config = new TrajectoryConfig(3.97350993, 2);
    
    config.setKinematics(getKinematics());
    
    //final String trajectoryJSON = "paths/YourPath.wpilib.json";
    try {
      final Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (final IOException ex) {
      SmartDashboard.putString("Unable to open trajectory: ","");
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
        RamseteCommand command = new RamseteCommand(
          trajectory,
          drive::getPose,
          disabledRamsete,//new RamseteController(2.0, 0.7),
          getFeedForward(),
          getKinematics(),
          drive::getWheelSpeeds,
          leftController,
          rightController,
          (leftVolts, rightVolts) -> {drive.tankVolts(leftVolts, rightVolts);},//m_driveSubsystem::set,
          drive);
      return command;
}

  
  public DifferentialDriveKinematics getKinematics() {
   return kinematics;
  }
  
  public SimpleMotorFeedforward getFeedForward() {
    return feedForward;
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

        
