package robot.OI;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector {

	public enum RobotPosition {LEFT, RIGHT, CENTER};
	public enum RobotBehaviour {SWITCH, SCALE, BASELINE};
	public enum SwitchOrScaleSide {LEFT, RIGHT};

	
	public SendableChooser<String> robotPosition;
	public SendableChooser<String> robotBehaviour;
	
	public AutoSelector(){
		
		//setting robot positions
		robotPosition = new SendableChooser<String>();
		robotPosition.addDefault("left", "left");
		robotPosition.addDefault("right", "right");
		robotPosition.addDefault("center", "center");
		
		//setting autonomous behaviour
		robotBehaviour = new SendableChooser<String>();
		robotBehaviour.addDefault("Place cube on switch", "switch");
		robotBehaviour.addDefault("Place cube on scale", "scale");
		robotBehaviour.addDefault("Cross baseline", "baseline");
		
	}
	
	public RobotPosition getRobotPosition(){
		switch(robotPosition.getSelected()){
			case "left": return RobotPosition.LEFT;
			case "right": return RobotPosition.RIGHT;
			case "center": return RobotPosition.CENTER;
			default:	return RobotPosition.LEFT;
		}
	}

	public RobotBehaviour getRobotBehaviour(){
		switch(robotPosition.getSelected()){
			case "switch": return RobotBehaviour.SWITCH;
			case "scale": return RobotBehaviour.SCALE;
			case "baseline": return RobotBehaviour.BASELINE;
			default:	return RobotBehaviour.BASELINE;
		}
	}

	public void updateSmartDashboard(){
		SmartDashboard.putData("Robot Position", robotPosition);
		SmartDashboard.putData("Robot Behaviour", robotBehaviour);
	}
	
}
