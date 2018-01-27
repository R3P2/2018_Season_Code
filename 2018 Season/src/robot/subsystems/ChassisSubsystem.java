package robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.RobotMap;
import robot.commands.JoystickCommand;

/**
 *
 */
public class ChassisSubsystem extends Subsystem {

	DoubleSolenoid PistonOne = new DoubleSolenoid(0, 1);
	DoubleSolenoid PistonTwo = new DoubleSolenoid(2, 3);

	// Our talon speed controlers. Only uncomment when talons are connected:
	// TalonSRX leftMotor = new TalonSRX(1);
	// TalonSRX rightMotor = new TalonSRX(0);

	Victor leftMotor = new Victor(RobotMap.LEFT_MOTOR_PORT);
	Victor rightMotor = new Victor(RobotMap.RIGHT_MOTOR_PORT);

	Encoder leftEncoder = new Encoder(RobotMap.LEFT_ENCODER_PORT_ONE, RobotMap.LEFT_ENCODER_PORT_TWO);
	Encoder rightEncoder = new Encoder(RobotMap.RIGHT_ENCODER_PORT_ONE, RobotMap.RIGHT_ENCODER_PORT_TWO);

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new JoystickCommand());
		leftMotor.setInverted(true);
	}

	public void setPistons(boolean state) {
		PistonOne.set(state ? Value.kForward : Value.kReverse);
		PistonTwo.set(state ? Value.kForward : Value.kReverse);
	}

	// These are the movement related methods
	// Use these to move the robot
	
	public void setMotors(double rightSpeed, double leftSpeed) {
		leftMotor.set(getPID(leftSpeed, leftEncoder.getDistance(), RobotMap.LEFT_ENCODER_MAX_SPEED));
		rightMotor.set(getPID(leftSpeed, rightEncoder.getDistance(), RobotMap.RIGHT_ENCODER_MAX_SPEED));
	}

	public void setSpeed(double speed) {
		setMotors(speed, speed);
	}

	public void setMovement(double speed, double turn) {
		if (Math.abs(turn) < 0.2 && Math.abs(speed) > 0.1) {

			setMotors(speed, speed);

			System.out.println("running");
		} else if (Math.abs(turn) > 0.2 && Math.abs(speed) > 0.1) {

			if (turn > 0.2) {
				setMotors(speed, (1 - turn) * speed);
			} else {
				setMotors((1 - Math.abs(turn)) * speed, speed);
			}

		} else if (Math.abs(turn) > 0.2) {

			setMotors(-turn, turn);

		} else {
			setSpeed(0);
		}
	}

	public double getPID(double speed, double feedback, int maxSpeed) {

		double normalizedFeedback = feedback / maxSpeed;

		if (normalizedFeedback > 1.0) {
			normalizedFeedback = 1.0;
		}
		if (normalizedFeedback < -1.0) {
			normalizedFeedback = -1.0;
		}

		double error = speed - normalizedFeedback;

		double output = RobotMap.KP * error;

		double normalizedOutput = output + speed;

		if (normalizedOutput > 1.0) {
			normalizedOutput = 1.0;
		}
		if (normalizedOutput < -1.0) {
			normalizedOutput = -1.0;
		}

		return normalizedOutput;
	}

}
