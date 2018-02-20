package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.RobotMap;
import robot.subsystems.ChassisSubsystem;

/**
 *
 */
public class DriveStraightWithGyro extends Command {

	ChassisSubsystem chassisSubsystem;
	private double timeout;
	private double distance;
	private double speed;
	
    public DriveStraightWithGyro(double distance, double speed) {
        requires(Robot.chassisSubsystem);
        this.distance = distance;
        this.speed = speed;
        timeout = RobotMap.TIME_OUT;
    }
    
    public DriveStraightWithGyro(double timeout, double distance, double speed) {
        requires(Robot.chassisSubsystem);
        this.distance = distance;
        this.timeout = timeout;
        this.speed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	chassisSubsystem = Robot.chassisSubsystem;
    	chassisSubsystem.gyro.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
  
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
