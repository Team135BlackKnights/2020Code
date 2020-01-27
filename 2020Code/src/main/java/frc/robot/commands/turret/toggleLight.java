package frc.robot.commands.turret;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

public class toggleLight extends Command {
	//This command when ran will set the hatch manipulator to the state it is currently not at
	private static boolean lightOn = false;
	
	public toggleLight(boolean lightOn) {
	}

  protected void execute() 
  {
  lightOn = !lightOn;
		Robot.turret.setLight(lightOn);
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
