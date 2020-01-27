package frc.robot.commands.drive;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class toggleCompressor extends InstantCommand {
	private static boolean isCompressorOn = true;
	// This is a little command that when is ran will set the compressor to the state that it is not currently set at
	public toggleCompressor() {
	}

	protected void execute() {
		isCompressorOn = !isCompressorOn;
		if (isCompressorOn) {
			Robot.drive.setCompressorOn();
		} else {
			Robot.drive.setCompressorOff();
		}
	}
}
