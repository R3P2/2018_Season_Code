/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package OI;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
		
	GameController gameController = new XBoxController(0);

	public boolean enableTurbo(){
		return gameController.isRightBumperDown();
	}
	
	public boolean disableTurbo() {
		return gameController.isLeftBumperDown();
	}
	
	public double getSpeed(){
		return gameController.getLeftYAxis();
	}
	
	public double getTurn(){
		return gameController.getRightXAxis();
	}
	
	public double getClimb(){
		return gameController.getRightYAxis();
	}
	
	public boolean isAccelerating(){
		return gameController.isADown();
	}
	
	public boolean isNotAccelerating(){
		return gameController.isXDown();
	}
}
