package robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.RobotMap;
import robot.commands.JoystickCommand;
import robot.util.Gyro;

/**
 *
 */
public class ChassisSubsystem extends Subsystem {

	// DoubleSolenoid PistonOne = new DoubleSolenoid(0, 1);
	DoubleSolenoid PistonTwo = new DoubleSolenoid(2, 3);

	// Our talon speed controlers. Only uncomment when talons are connected:
	// TalonSRX leftMotor = new TalonSRX(1);
	// TalonSRX rightMotor = new TalonSRX(0);

	// Our Victor speed controllers. Only uncomment when victors are connected:
	Victor leftMotor = new Victor(0);
	Victor rightMotor = new Victor(1);

	Encoder leftEncoder = new Encoder(0, 1);
	Encoder rightEncoder = new Encoder(2, 3, true);

	public Gyro gyro = new Gyro();

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new JoystickCommand());
		leftMotor.setInverted(true);
	}

	public void setPistons(boolean state) {
		// PistonOne.set(state ? Value.kForward : Value.kReverse);
		PistonTwo.set(state ? Value.kForward : Value.kReverse);
	}

	public double getEncoderCounts() {

		return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;

	}

	public void resetEncoders() {

		leftEncoder.reset();
		rightEncoder.reset();

	}

	public void setArmSpeed(double speed) {

		leftMotor.set(speed);
		rightMotor.set(speed);

	}

	public void setMotors(double rightSpeed, double leftSpeed) {
		leftMotor.set(movePid(leftSpeed, leftEncoder.getDistance(), RobotMap.MAX_LEFT_ENCODER_SPEED));
		rightMotor.set(movePid(rightSpeed, rightEncoder.getDistance(), RobotMap.MAX_LEFT_ENCODER_SPEED));
	}

	public void setMotors(double speed) {
		setMotors(speed, speed);
	}

	public void setMovement(double speed, double turn) {

		if (Math.abs(turn) < 0.2 && Math.abs(speed) > 0.1) {

			setMotors(speed, speed);

		} else if (Math.abs(turn) > 0.2 && Math.abs(speed) > 0.1) {

			if (turn > 0.2) {
				setMotors(speed, (1 - turn) * speed);
			} else {
				setMotors((1 - Math.abs(turn)) * speed, speed);
			}

		} else if (Math.abs(turn) > 0.2) {

			setMotors(-turn, turn);

		} else {
			setMotors(0);
		}

	}

	public double movePid(double speed, double feedback, double maxSpeed) {

		double normalizedFeedback = feedback / maxSpeed;
		if (normalizedFeedback > 1.0) {
			normalizedFeedback = 1.0;
		}
		if (normalizedFeedback < -1.0) {
			normalizedFeedback = -1.0;
		}

		double error = speed - normalizedFeedback;

		double output = speed + error;

		double normalizedOutput = output;

		if (normalizedOutput > 1.0) {
			normalizedOutput = 1.0;
		}
		if (normalizedOutput < -1.0) {
			normalizedOutput = -1.0;
		}

		return normalizedOutput;

	}

}

