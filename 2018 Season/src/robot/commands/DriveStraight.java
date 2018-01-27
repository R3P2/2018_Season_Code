package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */
public class DriveStraight extends Command {

	private double distance;
	private double speed;
	private double time;

    public DriveStraight(double distance,double speed,double time) {
        requires(Robot.chassisSubsystem);
        this.distance = distance;
        this.speed = speed;
        this.time = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.chassisSubsystem.setMotors(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.chassisSubsystem.setMotors(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.chassisSubsystem.getEncoderCounts() >= distance || timeSinceInitialized() >= time;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
