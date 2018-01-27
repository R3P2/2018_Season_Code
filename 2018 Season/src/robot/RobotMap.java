/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public final static int LEFT_MOTOR_PORT = 0;
	public final static int RIGHT_MOTOR_PORT = 0;
	
	public final static int LEFT_ENCODER_PORT_ONE = 0;
	public final static int LEFT_ENCODER_PORT_TWO = 1;
	public final static int RIGHT_ENCODER_PORT_ONE = 2;
	public final static int RIGHT_ENCODER_PORT_TWO = 3;
	
	public final static double KP = 0.1;
	
	public final static int MAX_LEFT_ENCODER_SPEED = 2000;
	public final static int MAX_RIGHT_ENCODER_SPEED = 1800;
	
	public final static double threshHold = 0.2;
	
}
