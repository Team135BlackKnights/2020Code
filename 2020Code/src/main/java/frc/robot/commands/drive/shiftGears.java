package frc.robot.commands.drive;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

public class shiftGears extends Command {
	//This command when ran will set the hatch manipulator to the state it is currently not at
	private static boolean solenoidPosition = true;
	
	public shiftGears(boolean isHighGear) {
	}

	protected void execute() {
		solenoidPosition = !solenoidPosition;
		Robot.drive.shiftGears(solenoidPosition);
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

	protected void end() {
	}

	protected void interrupted() {
		end();
	}
}
