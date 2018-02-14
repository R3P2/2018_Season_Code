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

	
	public final static int LEFT_MOTOR_PORT_ONE = 1;
	public final static int LEFT_MOTOR_PORT_TWO = 2;
	public final static int RIGHT_MOTOR_PORT_ONE = 3;
	public final static int RIGHT_MOTOR_PORT_TWO = 4;
	
	public final static int ARM_LIFT_MOTOR_PORT = 6;
	public final static int INTAKE_MOTOR_ONE_PORT = 7;
	public final static int INTAKE_MOTOR_TWO_PORT = 8;
	
	public final static int SOLENOID_PORT_ONE = 0;
	public final static int SOLENOID_PORT_TWO= 1;
	
	public final static double KP = 0.1;
	
	public final static int MAX_LEFT_ENCODER_SPEED = 2000;
	public final static int MAX_RIGHT_ENCODER_SPEED = 1800;
	
	public final static double JOYSTICK_NOISE_THRESHOLD = 0.2;
	
}
