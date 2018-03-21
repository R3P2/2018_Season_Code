package robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.torontocodingcollective.pid.TSpeedPID;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.RobotMap;
import robot.commands.JoystickCommand;
import robot.util.Gyro;

public class ChassisSubsystem extends Subsystem {

	// TODO: look at pids
	// TODO: test auto
	DoubleSolenoid pancakeShifter = new DoubleSolenoid(0, 1);

	TalonSRX leadLeft = new TalonSRX(RobotMap.LEFT_MOTOR_PORT_ONE);
	TalonSRX followLeft = new TalonSRX(RobotMap.LEFT_MOTOR_PORT_TWO);

	TalonSRX leadRight = new TalonSRX(RobotMap.RIGHT_MOTOR_PORT_ONE);
	TalonSRX followRight = new TalonSRX(RobotMap.RIGHT_MOTOR_PORT_TWO);

	TalonSRX armLiftMotor = new TalonSRX(RobotMap.ARM_LIFT_MOTOR_PORT);
	TalonSRX intakeMotor_One = new TalonSRX(RobotMap.INTAKE_MOTOR_ONE_PORT);
	TalonSRX intakeMotor_Two = new TalonSRX(RobotMap.INTAKE_MOTOR_TWO_PORT);

	Victor climbMotor = new Victor(9);

	Encoder rightMotorEncoder = new Encoder(2, 3, true);

	double m_dPrevSpeed = 0;
	double m_dAccelTime_s = 1.0;
	double m_dCycleTime_s = 0.020;
	int m_nAccelCount = 0;
	int m_nAccelCycles = (int) (m_dAccelTime_s / m_dCycleTime_s);
	double m_dSpeedAdvInc = 1.0 / m_nAccelCycles;

	public Gyro gyro = new Gyro();

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new JoystickCommand());

	}

	public void chassisInit() {

		followLeft.follow(leadLeft);
		followRight.follow(leadRight);

		leadLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		leadRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

		leadLeft.selectProfileSlot(0, 0); // First param is the slot,
											// second parameter is zero (For
											// a primary PID loop)
		leadLeft.config_kF(0, 0.0, RobotMap.K_TIMEOUT_MS);
		leadLeft.config_kP(0, 0.2, RobotMap.K_TIMEOUT_MS);
		leadLeft.config_kI(0, 0.0, RobotMap.K_TIMEOUT_MS);
		leadLeft.config_kD(0, 0.0, RobotMap.K_TIMEOUT_MS);

		leadRight.selectProfileSlot(0, 0); // First param is the slot,
		// second parameter is zero (For
		// a primary PID loop)
		leadRight.config_kF(0, 0.0, RobotMap.K_TIMEOUT_MS);
		leadRight.config_kP(0, 0.2, RobotMap.K_TIMEOUT_MS);
		leadRight.config_kI(0, 0.0, RobotMap.K_TIMEOUT_MS);
		leadRight.config_kD(0, 0.0, RobotMap.K_TIMEOUT_MS);

		intakeMotor_One.setInverted(true);
	}

	public void teleopInit() {

	}

	public void setArmLiftSpeed(double speed) {
		if (Math.abs(speed) > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
			armLiftMotor.set(ControlMode.PercentOutput, speed);
		} else {
			armLiftMotor.set(ControlMode.PercentOutput, 0);
		}
	}

	public void setIntakeSpeed(double speed) {
		if (Math.abs(speed) > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
			intakeMotor_One.set(ControlMode.PercentOutput, speed);
			intakeMotor_Two.set(ControlMode.PercentOutput, speed);
		} else {
			intakeMotor_One.set(ControlMode.PercentOutput, 0);
			intakeMotor_Two.set(ControlMode.PercentOutput, 0);
		}
	}

	public void setTurbo(boolean state) {
		// PistonOne.set(state ? Value.kForward : Value.kReverse);
		pancakeShifter.set(state ? Value.kForward : Value.kReverse);
	}

	public double getLeftEncoderCounts() {
		return leadLeft.getSelectedSensorPosition(0);
	}

	public double getRightEncoderCounts() {
		// return rightMotor_One.getSelectedSensorPosition(0);
		return rightMotorEncoder.getDistance();
	}

	public double getRightEncoderRate() {
		return rightMotorEncoder.getRate();
	}

	public int getLeftEncoderRate() {
		return leadLeft.getSelectedSensorVelocity(0);
	}

	public double getEncoderCounts() {
		return (getLeftEncoderCounts() + getRightEncoderCounts()) / 2;
	}

	// public double getClimbEncoder() {
	// return climbEncoder.getDistance();
	// }

	// public void resetClimbEncoder() {
	// climbEncoder.reset();
	// }

	public void setClimbMotors(double speed) {
		// if (getClimbEncoder() < RobotMap.MAX_CLIMB_HEIGHT && speed > 0) {
		climbMotor.set(speed);
		// } else if ((getClimbEncoder() < 100 && speed < 0)) {
		// if ((getClimbEncoder() < RobotMap.MAX_CLIMB_HEIGHT &&
		// getClimbEncoder() > 100) && speed > 0) {
		// climbMotor.set(speed);
		// } else {
		// climbMotor.set(0);
		// }
		// }
	}

	private void setLeftMotors(double speed) {

		leadLeft.set(ControlMode.PercentOutput, speed);

	}

	private void setRightMotors(double speed) {

		leadRight.set(ControlMode.PercentOutput, speed);

	}

	public void setMotors(double rightSpeed, double leftSpeed) {

		// Talon Motors
		setLeftMotors(leftSpeed);
		setRightMotors(rightSpeed);
	}

	private void setMotors(double speed) {
		setMotors(speed, speed);
	}

	public void setMovement(double speed, double turn) {

		double dabs_speed = Math.abs(speed), dabs_turn = Math.abs(turn);

		if (dabs_turn < RobotMap.JOYSTICK_NOISE_THRESHOLD && dabs_speed > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
			setMotors(speed, speed);
		} else if (dabs_turn > RobotMap.JOYSTICK_NOISE_THRESHOLD && dabs_speed > RobotMap.JOYSTICK_NOISE_THRESHOLD) {

			if (turn > 0.0) {
				setMotors(speed, (1 - turn) * speed);
			} else {
				setMotors((1 - turn) * speed, speed);
			}

		} else if (dabs_turn > RobotMap.JOYSTICK_NOISE_THRESHOLD) {

			setMotors(-turn, turn);

		} else {
			setMotors(0);
		}

	}

	/*
	 * Speed filter procedure: RAMPS towards any change in speed using the
	 * m_dSpeedAdvInc. RAMPS towards any abrupt changes in direction the same
	 * way DOES NOT RAMP if the speed drops to ZERO
	 * 
	 * Changes on Feb 10, 2018: The ramp-up from 0 is taking too long and we
	 * think that
	 * 
	 */
	public double FilteredSpeed(double dJSSpeed) {
		double dInputSpeed, dOutputSpeed, diff;

		// If Joystick speed (JS) is < threshold dInputSpeed is ZERO
		// If there has been a direction change, we do NOT want to RAMP the
		// speed down and then start ramping up in the opposite direction.
		// Instead, we will STOP (drop dInputSpeed to ZERO) and in the
		// subsequent cycles, ramp towards the new speed in the other direction.

		if (Math.abs(dJSSpeed) < RobotMap.JOYSTICK_NOISE_THRESHOLD)
			dInputSpeed = 0.0;
		else
			dInputSpeed = dJSSpeed;

		// Take the difference between the new speed and the
		// the speed of the last cycle.
		diff = dInputSpeed - m_dPrevSpeed;

		// We want to move towards the current "dInputSpeed"
		// using the "m_dSpeedAdvInc"
		if (Math.abs(diff) <= m_dSpeedAdvInc) // We are within an acceptable
												// window
			dOutputSpeed = dInputSpeed; // So just stay there.
		else if (diff > 0) // We are speeding up
			dOutputSpeed = m_dPrevSpeed + m_dSpeedAdvInc;
		else // We are slowing down
			dOutputSpeed = m_dPrevSpeed - m_dSpeedAdvInc * 2;

		m_dPrevSpeed = dOutputSpeed;

		return dOutputSpeed;
	}

	public void setAcceleration(double speed, double turn) {

		double daccspeed, dabs_turn;
		boolean blMoving;

		daccspeed = FilteredSpeed(speed);
		if (Math.abs(daccspeed) > 0)
			blMoving = true;
		else
			blMoving = false;

		dabs_turn = Math.abs(turn);

		// Modified version of "setMovement" block
		// We do not want the speed cut off if it is below the noise threshold
		// because the first stage of the acceleration will produce speeds
		// below the noise.

		if (blMoving == true) {
			if (dabs_turn > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
				// We are moving AND turning.
				// We need the turn direction:
				if (turn < 0) {
					setMotors(daccspeed, (1 - turn) * daccspeed);
				} else {
					setMotors((1 - dabs_turn) * daccspeed, daccspeed);
				}
			} else {
				// Not turning. Just moving:
				setMotors(daccspeed, daccspeed);
			}
		} else {
			if (dabs_turn > RobotMap.JOYSTICK_NOISE_THRESHOLD) {
				// We are turning and NOT moving (rotating)
				setMotors(-turn * 0.4, turn * 0.4);
			} else {
				// Not doing anything:
				setMotors(0, 0);
			}
		}
	}

	public double movePid(double speed, double feedback, double maxSpeed) {

		double normalizedFeedback = feedback / maxSpeed;

		if (normalizedFeedback > 1.0) {
			normalizedFeedback = 0.95;
		}
		if (normalizedFeedback < -1.0) {
			normalizedFeedback = -0.95;
		}

		double error = (speed - normalizedFeedback) * RobotMap.KP;

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

	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		// SmartDashboard.putNumber("Climb Encoder", getClimbEncoder());
		SmartDashboard.putNumber("Left Encoder", getLeftEncoderCounts());
		SmartDashboard.putNumber("Right Encoder", getRightEncoderCounts());
		SmartDashboard.putNumber("Left Encoder Rate", getLeftEncoderRate());
		SmartDashboard.putNumber("Right Encoder Rate", getRightEncoderRate());

	}
}
