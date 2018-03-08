package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.RobotMap;
import robot.util.Gyro;

/**
 *
 */
public class TurnToAngle extends Command {

	Gyro gyro;
	double angle;
	double currentAngle;

	final double Min_Speed = 0.5;
	final double Max_Speed = 0.75;

	public TurnToAngle(double angle) {
		requires(Robot.chassisSubsystem);
		this.angle = angle;
		gyro = Robot.chassisSubsystem.gyro;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		gyro.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		currentAngle = gyro.getSplitAngle();

		double speed = ((Math.abs(angle) - Math.abs(currentAngle)) / 165);

		if (speed < Min_Speed) {
			speed = Min_Speed;
		}

		if (speed > Max_Speed) {
			speed = Max_Speed;
		}

		if (currentAngle < angle) {

			// The current angle is smaller than the target angle
			// We need to turn right to increase our angle to match the target
			// angle

			Robot.chassisSubsystem.setMovement(0, -speed);

		} else {

			// The current angle is larger than the target angle
			// We need to turn left to decrease our angle to match the target
			// angle

			Robot.chassisSubsystem.setAcceleration(0, speed);

		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {

		return timeSinceInitialized() >= RobotMap.TIME_OUT || ((currentAngle >= angle - 1 && currentAngle <= angle + 1));

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
