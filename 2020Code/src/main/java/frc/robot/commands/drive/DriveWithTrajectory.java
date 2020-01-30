/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.FalconDrive;

/**
 * Add your docs here.
 */
public class DriveWithTrajectory extends TimedCommand {
  double leftVolts = 0;
  double rightVolts = 0;
  Trajectory trajectory;
  DifferentialDriveOdometry m_odometry;
  public DriveWithTrajectory(final double timeout) {
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

    final double currentHeading = getHeading();
    final String trajectoryJSON = "paths/YourPath.wpilib.json";
    try {
      final Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      final Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (final IOException ex) {
      SmartDashboard.putString("Unable to open trajectory: ","");
  }
  //TODO:: create the stuff to follow the trajectory
  //https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/creating-following-trajectory.html
  RamseteController disabledRamsete = new RamseteController() {
    @Override
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
            double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
    }
};
final BiConsumer<Double , Double > m_output;
var leftController = new PIDController(.6, 0, 0);
var rightController = new PIDController(.6, 0, 0);
  final RamseteCommand ramseteCommand = new RamseteCommand(//trajectory, Robot.drive::getPose, disabledRamsete, null, 
        trajectory,
        Robot.drive::getPose,
        //two numbers of ramsete controller are adjustment variables first is any number not negative and second is between 0 and 1
        //new RamseteController(1, .5),
        disabledRamsete,
        //The below Constanst need to be found after the robot is built...
        //https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/characterizing-drive.html
/*
        new SimpleMotorFeedforward(RobotMap.DRIVE.ksVolts,
                                   RobotMap.DRIVE.kvVoltSecondsPerMeter,
                                   RobotMap.DRIVE.kaVoltSecondsSquaredPerMeter),
                                   */
        RobotMap.DRIVE.kDriveKinematics,
       // getWheelSpeeds(),
      //  leftController,
      //  rightController,
        
        //m_output,
        (leftVolts, rightVolts) -> {Robot.drive.tankVolts(leftVolts, rightVolts);}, FalconDrive 
        
        
        
        //Working version (Supposedly)
        //https://github.com/wpilibsuite/allwpilib/blob/master/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/RamseteCommand.java
    );

}

/*
@FunctionalInterface
interface thing {
  int add();
}

public int add(int x, int y) {
  return x + y;
}

public void tankVolts(BiConsumer<double leftVolts, double rightVolts>)
{
  Robot.drive.frontLeftFX.setVoltage(leftVolts);
  Robot.drive.rearLeftFX.setVoltage(leftVolts);
  Robot.drive.frontRightFX.setVoltage(rightVolts);
  Robot.drive.rearRightFX.setVoltage(rightVolts);
  Robot.drive.chassis.feed();
}

*/


public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  final double leftEncoderVelocity = (Robot.drive.getEncoderVelocity(Robot.drive.frontLeftFX) + Robot.drive.getEncoderVelocity(Robot.drive.rearLeftFX)) / 2;
  final double rightEncoderVelocity = (Robot.drive.getEncoderVelocity(Robot.drive.frontRightFX) + Robot.drive.getEncoderVelocity(Robot.drive.rearRightFX)) / 2;
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
