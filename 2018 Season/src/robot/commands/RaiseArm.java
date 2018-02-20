package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.RobotMap;
import robot.subsystems.ChassisSubsystem;

/**
 *
 */
public class RaiseArm extends Command {

	private double targetHeight;
	private double timeout;

	public RaiseArm(boolean isScale, double timeout) {
		requires(Robot.chassisSubsystem);
		if (isScale) {
			targetHeight = RobotMap.SCALE_HEIGHT;
		} else {
			targetHeight = RobotMap.SWITCH_HEIGHT;
		}
		
		this.timeout = timeout;
	}

	public RaiseArm(boolean isScale) {
		if (isScale) {
			targetHeight = RobotMap.SCALE_HEIGHT;
		} else {
			targetHeight = RobotMap.SWITCH_HEIGHT;
		}

		timeout = RobotMap.TIME_OUT;

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.chassisSubsystem.setArmLiftSpeed(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.chassisSubsystem.getLiftEncoder() > targetHeight) {
			Robot.chassisSubsystem
					.setArmLiftSpeed(-1 * (0.2 + (1 - Robot.chassisSubsystem.getClimbEncoder() / targetHeight)));
		} else {
			Robot.chassisSubsystem.setArmLiftSpeed(0.2 + (1 - Robot.chassisSubsystem.getClimbEncoder() / targetHeight));
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.chassisSubsystem.getLiftEncoder() >= targetHeight || timeSinceInitialized() >= timeout;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.chassisSubsystem.setArmLiftSpeed(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
