package robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.OI.AutoSelector.RobotBehaviour;
import robot.OI.AutoSelector.RobotPosition;
import robot.OI.AutoSelector.SwitchOrScaleSide;
import robot.Robot;
/**
 *
 */
public class AutoCommand extends CommandGroup {

	private final double BASELINE_DISTANCE = 10.00;
	
	//maximum time in seconds
	public final double fullTime = 2;
	
	//turning angles
	private double turnAngle = 45;
	
	//distances
	private final double autoSpeed = 0.75;
	private final double switchDistance = 0;
	private final double switchLength = 0;
	private final double fieldWidth = 0;
	private final double scaleDistance = 0;
	private final double platformLength = 0;
	
    public AutoCommand() {
    	
    	
//    	addSequential(new DriveStraight(BASELINE_DISTANCE, 0.25, fullTime));
//    	addSequential(new TurnToAngle(turnAngle));
//    	addSequential(new TurnToAngle(-2*turnAngle));
    	
    	RobotPosition 	robotPosition 	= Robot.oi.autoSelector.getRobotPosition();
    	RobotBehaviour 	robotBehaviour 	= Robot.oi.autoSelector.getRobotBehaviour();
    	SwitchOrScaleSide allianceSwitch = Robot.oi.getSwitchSide();
    	SwitchOrScaleSide allianceScale = Robot.oi.getScaleSide();
    	
    	switch(robotBehaviour){
    		case BASELINE:
    			
    			//goes forward and crosses baseline
    			addSequential(new DriveStraight(BASELINE_DISTANCE, autoSpeed, fullTime));
    			
    		case SWITCH:
    			
    			//drive up to switch
				addSequential(new DriveStraight(switchDistance, autoSpeed, fullTime/3.0));
				
				//Auto Strategy: go around switch if alliance's side is on the opposite side to avoid contact with robots
    			if(robotPosition == RobotPosition.CENTER){
    				if(allianceSwitch == SwitchOrScaleSide.LEFT){
    					
	    				//turn 90 degrees to the left
	    				turnAngle = -90;
	    				
    				}else{
    					
    					//turn 90 degrees to the right
    					turnAngle = 90;
    					
    				}
    				
    				//turn to desired angle, drive forward a bit then turn back to face switch
    				addSequential(new TurnToAngle(turnAngle));
    				addSequential(new DriveStraight(switchLength/2.0, autoSpeed/2.0, fullTime/3.0));
    				addSequential(new TurnToAngle(-turnAngle));
    				
    			}else if(robotPosition == RobotPosition.LEFT){
    				
    				//turn towards switch (right)
    				addSequential(new TurnToAngle(turnAngle));
    				
    				if(allianceSwitch == SwitchOrScaleSide.LEFT){
    					//drive until 1/2 foot away from switch (values received from proximity sensor)
    				}else{
    					addSequential(new DriveStraight(switchLength, autoSpeed, fullTime/3.0));
    					addSequential(new TurnToAngle(turnAngle));
    					addSequential(new DriveStraight(switchLength/3.5, autoSpeed, fullTime/3.0));
    					addSequential(new TurnToAngle(turnAngle));
    					//drive until 1/2 foot away from switch (values received from proximity sensor)
    				}
    				
    			}else if(robotPosition == RobotPosition.RIGHT){
    				
    				//turn towards switch (left)
    				addSequential(new TurnToAngle(-turnAngle));
    				if(allianceSwitch == SwitchOrScaleSide.LEFT){
    					addSequential(new DriveStraight(switchLength, autoSpeed, fullTime/3.0));
    					addSequential(new TurnToAngle(-turnAngle));
    					addSequential(new DriveStraight(switchLength/3.5, autoSpeed, fullTime/3.0));
    					addSequential(new TurnToAngle(-turnAngle));
    					//drive until 1/2 foot away from switch (values received from proximity sensor)
    				}else{
    					//drive until 1/2 foot away from switch (values received from proximity sensor)
    				}
    			}

				//add drop cube command for switch here
    		case SCALE:
				
    			if(robotPosition == RobotPosition.CENTER){
    				if(allianceScale == SwitchOrScaleSide.LEFT){
    					
	    				//turn 90 degrees to the left
	    				turnAngle = -90;
	    				
    				}else{
    					
    					//turn 90 degrees to the right
    					turnAngle = 90;
    					
    				}
    				//turn to desired angle, drive to the edge of the field, 
    				//drive up to the scale and turn towards it
    				addSequential(new TurnToAngle(turnAngle));
    				addSequential(new DriveStraight(fieldWidth/2.0, autoSpeed, fullTime/3.0));
    				addSequential(new TurnToAngle(-turnAngle));
    				addSequential(new DriveStraight(scaleDistance, autoSpeed, fullTime/3.0));
    				addSequential(new TurnToAngle(-turnAngle));

    			}else{
    				
    				//drive up to scale (right behind it)
    				addSequential(new DriveStraight(scaleDistance, autoSpeed, fullTime/3.0));
    				
    				if(robotPosition == RobotPosition.LEFT){
	    					    				
	    				if(allianceSwitch == SwitchOrScaleSide.LEFT){
	    					
	    					//drive right up to the switch and turn to the right 90 degrees
	    					addSequential(new DriveStraight(1.0, autoSpeed, 1.0));
		    				addSequential(new TurnToAngle(turnAngle));
		    				//drive until 1/2 foot away from scale (values received from proximity sensor)
		    				
	    				}else{
	    					
	    					//turn right, drive across alliance platform then turn left towards scale plate
	    					addSequential(new TurnToAngle(turnAngle));
	    					addSequential(new DriveStraight(platformLength, autoSpeed, fullTime/3.0));
	    					addSequential(new TurnToAngle(-turnAngle));
	    					//drive until 1/2 foot away from switch (values received from proximity sensor)
	    				}
	    				
	    			}else if(robotPosition == RobotPosition.RIGHT){
	    				
	    				if(allianceSwitch == SwitchOrScaleSide.RIGHT){
	    					
	    					//drive right up to the switch and turn to the left 90 degrees
	    					addSequential(new DriveStraight(1.0, autoSpeed, 1.0));
		    				addSequential(new TurnToAngle(-turnAngle));
		    				//drive until 1/2 foot away from scale (values received from proximity sensor)
		    				
	    				}else{
	    					
	    					//turn left, drive across alliance platform then turn right towards scale plate
	    					addSequential(new TurnToAngle(-turnAngle));
	    					addSequential(new DriveStraight(platformLength, autoSpeed, fullTime/3.0));
	    					addSequential(new TurnToAngle(turnAngle));
	    					//drive until 1/2 foot away from switch (values received from proximity sensor)
	    				}
	    			}
    			}

				//add drop cube command for scale here
    		
    		default:
    			addSequential(new DriveStraight(BASELINE_DISTANCE, autoSpeed, fullTime));
    	}
    	
    	
    	
        
    }
    
 
}
