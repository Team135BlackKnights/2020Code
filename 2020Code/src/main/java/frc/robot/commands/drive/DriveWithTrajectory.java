/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.FalconDrive;

/**
 * Add your docs here.
 */
public class DriveWithTrajectory extends TimedCommand {
  /**
   * Add your docs here.
   */
  DifferentialDriveOdometry m_odometry;
  public DriveWithTrajectory(double timeout) {
    super(timeout);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double currentHeading = getHeading();
    String trajectoryJSON = "paths/YourPath.wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      SmartDashboard.putString("Unable to open trajectory: ","");
  }
  //TODO:: create the stuff to follow the trajectory
  //https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/creating-following-trajectory.html
  
  RamseteCommand ramseteCommand = new RamseteCommand(
        trajectoryJSON,
        getPose(),
        //two numbers of ramsete controller are adjustment variables first is any number not negative and second is between 0 and 1
        new RamseteController(1, .5),
        //The below Constanst need to be found after the robot is built...
        //https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/characterizing-drive.html

        new SimpleMotorFeedforward(RobotMap.DRIVE.ksVolts,
                                   RobotMap.DRIVE.kvVoltSecondsPerMeter,
                                   RobotMap.DRIVE.kaVoltSecondsSquaredPerMeter),
        RobotMap.DRIVE.kDriveKinematics,
        getWheelSpeeds(),
        new PIDController(.6, 0, 0),
        new PIDController(.6, 0, 0),
        // RamseteCommand passes volts to the callback
        Robot.drive::tankVolts,
        Robot.drive
    );


    
}

public void tankVolts(double leftVolts, double rightVolts)
{
  Robot.drive.frontLeftFX.setVoltage(leftVolts);
  Robot.drive.rearLeftFX.setVoltage(leftVolts);
  Robot.drive.frontRightFX.setVoltage(rightVolts);
  Robot.drive.rearRightFX.setVoltage(rightVolts);
  Robot.drive.chassis.feed();
}




public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  double leftEncoderVelocity = (Robot.drive.getEncoderVelocity(Robot.drive.frontLeftFX) + Robot.drive.getEncoderVelocity(Robot.drive.rearLeftFX)) / 2;
  double rightEncoderVelocity = (Robot.drive.getEncoderVelocity(Robot.drive.frontRightFX) + Robot.drive.getEncoderVelocity(Robot.drive.rearRightFX)) / 2;
  return new DifferentialDriveWheelSpeeds(leftEncoderVelocity, rightEncoderVelocity);
}

  public double getHeading()
  {
    return Robot.drive.getAngle();
  }

  // Called once after timeout
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
