package frc.robot.commands.drive.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auton.EncoderDriveToWithAngle;

public class AutoMaybe extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoMaybe() {
    addSequential(new EncoderDriveToWithAngle(-60, 60, 0, false));
    addSequential(new EncoderDriveToWithAngle(-38, 23, -110, false));
    //addSequential(new EncoderDriveToWithAngle(-60, 60, 0));
    //addSequential(new EncoderDriveToWithAngle(-60, 60, 0));

    //addSequential(new EncoderDrive(-20,20,2));
    //addSequential(new EncoderDrive(-100, 100, 2, true));
    //addSequential(new EncoderDrive(-39, 39, 2));
    //addSequential(new EncoderDrive(-51, 80, 2));
    //addSequential(new EncoderDrive(-84, 102, 2));
    //addSequential(new EncoderDrive(-75, 75, 2));


    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
