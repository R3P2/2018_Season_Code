package robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.Robot;
import robot.RobotMap;
import robot.commands.JoystickCommand;
import robot.util.Gyro;

/**
 *
 */
public class ChassisSubsystem extends Subsystem {

	DoubleSolenoid pancakeShifter = new DoubleSolenoid(1, 3);

	// Our talon speed controlers. Only uncomment when talons are connected:

	TalonSRX leftMotor_One = new TalonSRX(RobotMap.LEFT_MOTOR_PORT_ONE);
	TalonSRX leftMotor_Two = new TalonSRX(RobotMap.LEFT_MOTOR_PORT_TWO);

	TalonSRX rightMotor_One = new TalonSRX(RobotMap.RIGHT_MOTOR_PORT_ONE);
	TalonSRX rightMotor_Two = new TalonSRX(RobotMap.RIGHT_MOTOR_PORT_TWO);

	Victor climbMotor = new Victor(9);

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

		leftMotor_One.setInverted(true);
		leftMotor_Two.setInverted(true);

		leftMotor_One.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightMotor_One.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

	}

	public void setTurbo(boolean state) {
		// PistonOne.set(state ? Value.kForward : Value.kReverse);
		pancakeShifter.set(state ? Value.kForward : Value.kReverse);
	}

	public double getLeftEncoderCounts() {

		return leftMotor_One.getSelectedSensorPosition(0);

	}

	public double getRightEncoderCounts() {

		return rightMotor_One.getSelectedSensorPosition(0);

	}
	
	public double getEncoderCounts () {
		
		return (getLeftEncoderCounts() + getRightEncoderCounts()) / 2;
		
	}

	public void resetEncoders() {

		leftMotor_One.setSelectedSensorPosition(0, 0, 0);
		leftMotor_One.setSelectedSensorPosition(0, 0, 0);

	}

	public void setClimbMotors(double speed) {

		climbMotor.set(speed);

	}

	private void setLeftMotors(double speed) {

		leftMotor_One.set(ControlMode.PercentOutput, speed);
				//movePid(speed, leftEncoder.getDistance(), RobotMap.MAX_LEFT_ENCODER_SPEED));
		leftMotor_Two.set(ControlMode.PercentOutput, speed);
				//movePid(speed, rightEncoder.getDistance(), RobotMap.MAX_RIGHT_ENCODER_SPEED));

	}

	private void setRightMotors(double speed) {

		rightMotor_One.set(ControlMode.PercentOutput,speed);
				//movePid(speed, leftEncoder.getDistance(), RobotMap.MAX_LEFT_ENCODER_SPEED));
		rightMotor_Two.set(ControlMode.PercentOutput,speed);
				//movePid(speed, rightEncoder.getDistance(), RobotMap.MAX_RIGHT_ENCODER_SPEED));

	}

	private void setMotors(double rightSpeed, double leftSpeed) {

		// Talon Motors
		setLeftMotors(leftSpeed);
		setRightMotors(rightSpeed);
		// leftMotor.set(movePid(leftSpeed, leftEncoder.getDistance(),
		// RobotMap.MAX_LEFT_ENCODER_SPEED));
		// rightMotor.set(movePid(rightSpeed, rightEncoder.getDistance(),
		// RobotMap.MAX_RIGHT_ENCODER_SPEED));

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

		double dabs_speed, dabs_turn, daccspeed;

		dabs_speed = Math.abs(speed);
		dabs_turn = Math.abs(turn);

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

		if (dabs_turn < 0 && dabs_speed > 0) {
			setMotors(daccspeed, daccspeed);
		} else if (dabs_turn > 0 && dabs_speed > 0) {

			if (turn > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
				setMotors(daccspeed, (1 - turn) * daccspeed);
			} else {
				setMotors((1 - dabs_turn) * daccspeed, daccspeed);
			}
		} else if (dabs_turn > RobotMap.JOYSTICK_NOISE_THRESHOLD) {

			setMotors(-turn, turn);

		} else {
			setMotors(0);
		}
		
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
