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
		
		double elevatorSpeed = oi.getClimb();
		
		if(startAcceleration){isAccelerating = true;}
		if(stopAcceleration){isAccelerating = false;}
		
		if(isAccelerating){
			chassisSubsystem.setAcceleration(oi.getSpeed(), oi.getTurn());	
		}else{
			chassisSubsystem.setMovement(oi.getSpeed(), oi.getTurn());
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
