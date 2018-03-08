/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package OI;

import robot.RobotMap;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public AutoSelector autoSelector = new AutoSelector();

	GameController driveGameController = new XBoxController(0);
	GameController liftGameController = new XBoxController(1);

	public boolean enableTurbo() {
		return driveGameController.isRightBumperDown();
	}

	public boolean disableTurbo() {
		return driveGameController.isLeftBumperDown();
	}

	public boolean turnToAngle() {
		return driveGameController.isBDown();
	}

	public double getSpeed() {
		return -driveGameController.getLeftYAxis();
	}

	public double getTurn() {
		return driveGameController.getRightXAxis();
	}

	public double getClimb() {
		return driveGameController.getRightYAxis();
	}

	public boolean isAccelerating() {
		return driveGameController.isADown();
	}

	public boolean isNotAccelerating() {
		return driveGameController.isXDown();
	}

	public boolean resetEncoders() {
		return driveGameController.isYDown();
	}

	public double getClimbSpeed() {
		if (liftGameController.getLeftTriggerAxis() > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
			return -liftGameController.getLeftTriggerAxis();
		} else if(liftGameController.getRightTriggerAxis() > RobotMap.JOYSTICK_NOISE_THRESHOLD){
			return liftGameController.getRightTriggerAxis();
		}else{
			return 0;
		}
	}


	public double getliftSpeed(){
		return liftGameController.getLeftYAxis() * 0.8;
	}

	
	public double getIntakeSpeed(){
		return liftGameController.getRightYAxis() * 0.8;
	}
	
}
