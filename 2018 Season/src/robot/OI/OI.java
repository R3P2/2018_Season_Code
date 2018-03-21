/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.OI;

import edu.wpi.first.wpilibj.DriverStation;
import robot.RobotMap;
import robot.OI.AutoSelector.SwitchOrScaleSide;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	String gameData = DriverStation.getInstance().getGameSpecificMessage();
	
	public AutoSelector autoSelector = new AutoSelector();

	GameController driveGameController = new XBoxController(0);
	GameController liftGameController = new XBoxController(1);

	/**
	 * @return
	 * true = right
	 * false = left
	 */
	
	public SwitchOrScaleSide getSwitchSide(){
		if (gameData.length() > 0) {
			if (gameData.charAt(0) == 'L') {
				return SwitchOrScaleSide.LEFT;
			} else {
				return SwitchOrScaleSide.RIGHT;
			}
		}
		return SwitchOrScaleSide.LEFT;
	}
	
	public SwitchOrScaleSide getScaleSide(){
		if (gameData.length() > 0) {
			if (gameData.charAt(1) == 'L') {
				return SwitchOrScaleSide.LEFT;
			} else {
				return SwitchOrScaleSide.RIGHT;
			}
		}
		return SwitchOrScaleSide.LEFT;
	}
	
	public boolean enableTurbo() {
		return driveGameController.isRightBumperDown();
	}

	public boolean disableTurbo() {
		return driveGameController.isLeftBumperDown();
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

	// public boolean resetEncoders() {
	// return driveGameController.isYDown();
	// }

	public double getClimbSpeed() {
		if (liftGameController.getRightTriggerAxis() > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
			return liftGameController.getRightTriggerAxis();
		}

		if (liftGameController.getLeftTriggerAxis() > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
			return -liftGameController.getLeftTriggerAxis();
		}

		return 0;

	}

	public boolean liftUp() {
		return liftGameController.isYDown();
	}

	public boolean liftDown() {
		return liftGameController.isADown();
	}

	public boolean intakeIn() {
		return liftGameController.isBDown();
	}

	public boolean intakeOut() {
		return liftGameController.isXDown();
	}

}
