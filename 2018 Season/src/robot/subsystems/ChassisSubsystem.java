package robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
	 TalonSRX leftMotor1 = new TalonSRX(1);
	 TalonSRX leftMotor2 = new TalonSRX(0);
	 TalonSRX rightMotor1 = new TalonSRX(2);
	 TalonSRX rightMotor2 = new TalonSRX(3);
	// Our Victor speed controllers. Only uncomment when victors are connected:
//	Victor leftMotor = new Victor(0);
//	Victor rightMotor = new Victor(1);

	Encoder leftEncoder = new Encoder(0, 1);
	Encoder rightEncoder = new Encoder(2, 3, true);

	double m_dPrevSpeed = 0;
	double m_dAccelTime_s = 4.0;
	double m_dCycleTime_s = 0.020;
	int m_nAccelCount = 0;
	int m_nAccelCycles = (int) (m_dAccelTime_s / m_dCycleTime_s);
	double m_dSpeedAdvInc = 1.0 / m_nAccelCycles;

	public Gyro gyro = new Gyro();

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new JoystickCommand());
		leftMotor1.setInverted(true);
		leftMotor2.setInverted(true);
	}

	public void setTurbo(boolean state) {
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

	private void setMotors(double rightSpeed, double leftSpeed) {
		//Talon Motors
		leftMotor1.set(ControlMode.PercentOutput, movePid(leftSpeed, leftEncoder.getDistance(), RobotMap.MAX_LEFT_ENCODER_SPEED));
		leftMotor2.set(ControlMode.PercentOutput, movePid(leftSpeed, leftEncoder.getDistance(), RobotMap.MAX_LEFT_ENCODER_SPEED));
		rightMotor1.set(ControlMode.PercentOutput, movePid(rightSpeed, rightEncoder.getDistance(), RobotMap.MAX_RIGHT_ENCODER_SPEED));
		rightMotor2.set(ControlMode.PercentOutput, movePid(rightSpeed, rightEncoder.getDistance(), RobotMap.MAX_RIGHT_ENCODER_SPEED));
//		leftMotor.set(movePid(leftSpeed, leftEncoder.getDistance(), RobotMap.MAX_LEFT_ENCODER_SPEED));
//		rightMotor.set(movePid(rightSpeed, rightEncoder.getDistance(), RobotMap.MAX_RIGHT_ENCODER_SPEED));
	}

	private void setMotors(double speed) {
		setMotors(speed, speed);
	}

	public void setMovement(double speed, double turn) {

		double dabs_speed, dabs_turn;

		dabs_speed = Math.abs(speed);
		dabs_turn = Math.abs(turn);

		if (dabs_turn < RobotMap.JOYSTICK_NOISE_THRESHOLD && dabs_speed > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
			setMotors(speed, speed);
		} else if (dabs_turn > RobotMap.JOYSTICK_NOISE_THRESHOLD && dabs_speed > RobotMap.JOYSTICK_NOISE_THRESHOLD) {

			if (turn > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
				setMotors(speed, (1 - turn) * speed);
			} else {
				setMotors((1 - dabs_turn) * speed, speed);
			}
		} else if (dabs_turn > RobotMap.JOYSTICK_NOISE_THRESHOLD) {

			setMotors(-turn, turn);

		} else {
			setMotors(0);
		}

	}

	public void setAcceleration(double speed, double turn) {

		double dabs_speed, daccspeed;

		dabs_speed = Math.abs(speed);

		if (m_dPrevSpeed < RobotMap.JOYSTICK_NOISE_THRESHOLD) {
			if (dabs_speed < RobotMap.JOYSTICK_NOISE_THRESHOLD) {
				m_nAccelCount = 0;
				daccspeed = 0;
			} else {
				m_nAccelCount++;
				daccspeed = m_nAccelCount * m_dSpeedAdvInc;
				if (speed < 0)
					daccspeed = -daccspeed;
			}
		} else {
			if (m_nAccelCount < m_nAccelCycles) {
				m_nAccelCount++;
				daccspeed = m_nAccelCount * m_dSpeedAdvInc;
				if (speed < 0)
					daccspeed = -daccspeed;
				if (Math.abs(daccspeed) > dabs_speed) {
					daccspeed = speed;
				}
			} else {
				daccspeed = speed;
			}
		}

		setMovement(daccspeed, turn);

		m_dPrevSpeed = dabs_speed;
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
