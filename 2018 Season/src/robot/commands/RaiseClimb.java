package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.RobotMap;
import robot.subsystems.ChassisSubsystem;

/**
 *
 */
public class RaiseClimb extends Command {

	ChassisSubsystem chassisSubsystem;
	private double timeout;
	
	public RaiseClimb(double angle) {
		requires(Robot.chassisSubsystem);
		chassisSubsystem = Robot.chassisSubsystem;
		timeout = RobotMap.TIME_OUT;
	}
	
	public RaiseClimb(double angle, double timeout) {
		requires(Robot.chassisSubsystem);
		chassisSubsystem = Robot.chassisSubsystem;
		this.timeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		chassisSubsystem.setClimbMotors(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		chassisSubsystem.setClimbMotors(0.2 + (1 - chassisSubsystem.getClimbEncoder() / RobotMap.MAX_CLIMB_HEIGHT));

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {

		return chassisSubsystem.getClimbEncoder() >= RobotMap.MAX_CLIMB_HEIGHT || timeSinceInitialized() >= timeout;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.chassisSubsystem.setMovement(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
