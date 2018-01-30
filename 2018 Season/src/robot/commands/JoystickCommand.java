package robot.commands;

import OI.OI;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.ChassisSubsystem;

/**
 *
 */
public class JoystickCommand extends Command {

	ChassisSubsystem chassisSubsystem;
	OI oi;
	boolean isAccelerating;

	public JoystickCommand() {
		requires(Robot.chassisSubsystem);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		chassisSubsystem = Robot.chassisSubsystem;
		oi = Robot.oi;
		isAccelerating = true;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		boolean startAcceleration = oi.isAccelerating();
		boolean stopAcceleration = oi.isNotAccelerating();

		boolean enableTurbo = oi.enableTurbo();
		boolean disableTurbo = oi.disableTurbo();
		
		double speed = oi.getSpeed();
		double turn = oi.getTurn();
		
		if (startAcceleration) {
			isAccelerating = true;
		}
		
		if (stopAcceleration) {
			isAccelerating = false;
		}

		if (enableTurbo) {
			chassisSubsystem.setTurbo(true);
		} else if (disableTurbo) {
			chassisSubsystem.setTurbo(false);
		}
		
		if (isAccelerating) {
			chassisSubsystem.setAcceleration(speed, turn);
		} else {
			chassisSubsystem.setMovement(speed, turn);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
