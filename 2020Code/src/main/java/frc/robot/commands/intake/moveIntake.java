package frc.robot.commands.intake;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

public class moveIntake extends Command {
	//This command when ran will set the hatch manipulator to the state it is currently not at
	private static boolean solenoidPosition = true;
	
	public moveIntake(boolean pistonsInOut) {
	}

	protected void execute() {
		solenoidPosition = !solenoidPosition;
		Robot.intake.raiseLower(solenoidPosition);
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
