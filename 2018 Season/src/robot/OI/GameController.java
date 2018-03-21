package robot.OI;

import edu.wpi.first.wpilibj.Joystick;

public abstract class GameController {

	int port;
	
	Joystick joystick;
	
	public GameController (int port) {
		
		this.port = port;
		joystick = new Joystick(port);
		
	}
	
	public abstract boolean isADown();	
	public abstract boolean isBDown();	
	public abstract boolean isXDown();	
	public abstract boolean isYDown();	
	public abstract boolean isLeftBumperDown();	
	public abstract boolean isRightBumperDown();	
	public abstract boolean isButton1Down();	
	public abstract boolean isButton2Down();	
	public abstract boolean isLeftThumbDown();
	public abstract boolean isRightThumbDown();
	
	public abstract double getLeftXAxis();
	public abstract double getLeftYAxis();	
	public abstract double getLeftTriggerAxis();
	public abstract double getRightTriggerAxis();
	public abstract double getRightXAxis();
	public abstract double getRightYAxis();
	
}
