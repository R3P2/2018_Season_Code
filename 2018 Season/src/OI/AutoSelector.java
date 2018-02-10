package OI;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector {

	public enum RobotPosition {LEFT, RIGHT, CENTER};
	public enum RobotBehaviour {SWITCH, SCALE, BASELINE};
	public enum AllianceSwitch {LEFT, RIGHT};
	public enum AllianceScale {LEFT, RIGHT};

	
	public SendableChooser<String> robotPosition;
	public SendableChooser<String> robotBehaviour;
	public SendableChooser<String> allianceSwitchSide;
	public SendableChooser<String> allianceScaleSide;
	
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
		
		//determining alliance side of switch
		allianceSwitchSide = new SendableChooser<String>();
		allianceSwitchSide.addDefault("right", "right");
		allianceSwitchSide.addDefault("left", "left");
		
		//determining alliance side of scale
		allianceScaleSide = new SendableChooser<String>();
		allianceScaleSide.addDefault("right", "right");
		allianceScaleSide.addDefault("left", "left");
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
	
	public AllianceSwitch getAllianceSwitch(){
		switch(robotPosition.getSelected()){
			case "left": return AllianceSwitch.LEFT;
			case "right": return AllianceSwitch.RIGHT;
			default:	return AllianceSwitch.LEFT;
		}
	}
	
	public AllianceScale getAllianceScale(){
		switch(robotPosition.getSelected()){
			case "left": return AllianceScale.LEFT;
			case "right": return AllianceScale.RIGHT;
			default:	return AllianceScale.LEFT;
		}
	}

	public void updateSmartDashboard(){
		SmartDashboard.putData("Robot Position", robotPosition);
		SmartDashboard.putData("Robot Behaviour", robotBehaviour);
		SmartDashboard.putData("Alliance Switch", allianceSwitchSide);
		SmartDashboard.putData("Alliance Scale", allianceScaleSide);

	}
	
}
