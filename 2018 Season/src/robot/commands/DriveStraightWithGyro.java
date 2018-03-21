package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.RobotMap;
import robot.subsystems.ChassisSubsystem;
import robot.util.Gyro;

/**
 *
 */
public class DriveStraightWithGyro extends Command {

	ChassisSubsystem chassisSubsystem;
	Gyro gyro;
	private double timeout;
	private double distance;
	private double speed;
	
    public DriveStraightWithGyro(double distance, double speed) {
        requires(Robot.chassisSubsystem);
        this.distance = distance * RobotMap.MAX_ENCODER_COUNTS_PER_FT;
        this.speed = speed;
        timeout = RobotMap.TIME_OUT;
    }
    
    public DriveStraightWithGyro(double timeout, double distance, double speed) {
        requires(Robot.chassisSubsystem);
        this.distance = distance * RobotMap.MAX_ENCODER_COUNTS_PER_FT;
        this.timeout = timeout;
        this.speed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	chassisSubsystem = Robot.chassisSubsystem;
    	gyro = chassisSubsystem.gyro;
    	gyro.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double rightSpeed = speed;
    	double leftSpeed = speed;
    	
    	double angle = gyro.getSplitAngle();
    	if(angle > 2){
    		rightSpeed = speed * 0.25;
    		leftSpeed = -speed * 0.25;
    	}else if (angle < -2){
    		rightSpeed = -speed * 0.25;
    		leftSpeed = speed * 0.25;
    	}
    	
    	Robot.chassisSubsystem.setMotors(rightSpeed, leftSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.chassisSubsystem.getEncoderCounts() >= distance || timeSinceInitialized() >= timeout;
    }

    // Called once after isFinished returns true
    protected void end() {
    	chassisSubsystem.setMotors(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
